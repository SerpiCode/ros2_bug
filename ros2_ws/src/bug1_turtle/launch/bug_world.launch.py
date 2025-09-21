from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Caminho para o mundo
    world_file = '/home/luisvviana/ros2_ws/src/bug1_turtle/worlds/bug1_world/bug1_world.world'

    # Caminho onde está a pasta models
    os.environ["GAZEBO_MODEL_PATH"] = "/home/luisvviana/ros2_ws/src/bug1_turtle/worlds"

    return LaunchDescription([
        # Inicia Gazebo com o mundo
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_file,
                '-s', 'libgazebo_ros_factory.so',
                '--pause'
            ],
            output='screen'
        ),

        # Spawna o TurtleBot3 Waffle no ponto inicial desejado
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'wafflebot',
                '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf',
                '-x', '1.540100',
                '-y', '-7.914520',
                '-z', '0.5'
            ],
            output='screen'
        ),

        # Inicializa o nó do algoritmo Bug1
        Node(
            package='bug1_turtle',
            executable='bug1_node',
            name='bug1_node',
            output='screen'
        ),
    ])
