import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Bug1Sensors(Node):
    def __init__(self):
        super().__init__('bug1_sensors')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.pose = None
        self.ranges = []

        # Estados
        self.state = "GOAL_SEEK"  # GOAL_SEEK, EVADE, TURN_LEFT, WALL_FOLLOW
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        # Parâmetros
        self.goal = (2.073590, 9.102220)
        self.linear_speed = 0.5
        self.angular_speed = 0.8
        self.evade_distance = 0.4
        self.safe_front_distance = 0.2
        self.desired_left_distance = 0.5
        self.min_left_distance = 0.15

        # Controle EVADE e TURN_LEFT
        self.evade_start_x = None
        self.evade_start_y = None
        self.turn_start_yaw = None
        self.turn_angle = 5*math.pi / 12  # 75° giro para a esquerda

        # M-line e closest point
        self.start_position = None
        self.hit_point = []
        self.closest_point = None
        self.is_finding_closest_point = False

        # Loop de controle
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose
        if self.start_position is None:
            self.start_position = (self.pose.position.x, self.pose.position.y)

    def scan_callback(self, msg: LaserScan):
        self.ranges = msg.ranges

    def enter_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"➡️ Entering state {new_state}")

    def control_loop(self):
        if self.pose is None or not self.ranges:
            return

        twist = Twist()

        # Processamento LIDAR
        n = len(self.ranges)
        front_ranges = [r for r in self.ranges[n//3:2*n//3] if not math.isinf(r)]
        left_ranges = [r for r in self.ranges[2*n//3:] if not math.isinf(r)]
        front_min = min(front_ranges) if front_ranges else 10.0
        left_min = min(left_ranges) if left_ranges else 10.0

        # Distância até o objetivo
        dx = self.goal[0] - self.pose.position.x
        dy = self.goal[1] - self.pose.position.y
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal < 0.2:
            self.get_logger().info("🎯 Goal reached!")
            self.cmd_pub.publish(Twist())
            return

        # Checa pausa de 0.5s ao entrar no estado
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.state_start_time < 0.5:
            self.cmd_pub.publish(Twist())
            return

        # ---------------- ESTADOS ----------------
        if self.state == "GOAL_SEEK":
            if front_min < self.safe_front_distance:
                # Inicia EVADE
                self.evade_start_x = self.pose.position.x
                self.evade_start_y = self.pose.position.y
                self.enter_state("EVADE")
            else:
                yaw = self.get_yaw()
                angle_to_goal = math.atan2(dy, dx)
                angle_error = self.angle_diff(angle_to_goal, yaw)
                twist.linear.x = self.linear_speed
                twist.angular.z = 1.0 * angle_error

        elif self.state == "EVADE":
            dx_evade = self.pose.position.x - self.evade_start_x
            dy_evade = self.pose.position.y - self.evade_start_y
            dist_evade = math.hypot(dx_evade, dy_evade)

            if dist_evade < self.evade_distance:
                twist.linear.x = -self.linear_speed
            else:
                self.turn_start_yaw = self.get_yaw()
                self.enter_state("TURN_LEFT")

        elif self.state == "TURN_LEFT":
            current_yaw = self.get_yaw()
            turned_angle = self.angle_diff(current_yaw, self.turn_start_yaw)
            if turned_angle < self.turn_angle:
                twist.angular.z = self.angular_speed
            else:
                self.hit_point.append((self.pose.position.x, self.pose.position.y))
                self.closest_point = (self.pose.position.x, self.pose.position.y)
                self.enter_state("WALL_FOLLOW")

        elif self.state == "WALL_FOLLOW":
            # Atualiza closest point
            if not self.is_finding_closest_point:
                if self.distance_to_goal(self.pose_to_tuple(), self.goal) < self.distance_to_goal(self.closest_point, self.goal):
                    self.closest_point = self.pose_to_tuple()

            # Ajuste de distância da parede
            error = self.desired_left_distance - left_min
            twist.linear.x = self.linear_speed
            twist.angular.z = max(min(0.8 * error, 0.7), -0.7)

            # Se encostar muito perto da parede
            if left_min < self.min_left_distance:
                self.evade_start_x = self.pose.position.x
                self.evade_start_y = self.pose.position.y
                self.enter_state("EVADE")

            # Retorna à M-line se encontrou closest point
            if self.is_finding_closest_point:
                if self.distance_to_goal(self.pose_to_tuple(), self.closest_point) < 0.2:
                    if self.is_on_M_line(self.pose.position.x, self.pose.position.y, self.start_position[0], self.start_position[1]):
                        self.enter_state("GOAL_SEEK")
                        self.hit_point = []
                        self.closest_point = None
                        self.is_finding_closest_point = False

            # Retorna ao GOAL_SEEK se caminho livre
            if front_min > self.safe_front_distance * 2 and left_min > self.desired_left_distance * 2:
                self.enter_state("GOAL_SEEK")

        self.cmd_pub.publish(twist)

    # -------- Funções auxiliares --------
    def get_yaw(self):
        q = self.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2*math.pi
        while diff < -math.pi:
            diff += 2*math.pi
        return diff

    def pose_to_tuple(self):
        return (self.pose.position.x, self.pose.position.y)

    def distance_to_goal(self, p1, p2):
        if p1 is None or p2 is None:
            return float('inf')
        return math.hypot(p2[0]-p1[0], p2[1]-p1[1])

    def is_on_M_line(self, x_current, y_current, x0, y0, threshold=0.05):
        goal_x, goal_y = self.goal
        if x0 - goal_x != 0:
            dist = abs(y_current - ((x_current - x0)*(y0 - goal_y)/(x0 - goal_x) + y0))
            return dist < threshold
        else:
            dist = abs(x_current - goal_x)
            return dist < threshold


def main(args=None):
    rclpy.init(args=args)
    node = Bug1Sensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
