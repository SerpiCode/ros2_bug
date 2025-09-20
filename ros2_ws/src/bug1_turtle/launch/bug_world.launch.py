from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Caminho para o mundo
    world_file = '/home/luisvviana/ros2_ws/src/bug1_turtle/worlds/bug1_world/bug1_world.world'

    # Configura o GAZEBO_MODEL_PATH incluindo seus modelos e TurtleBot3
    os.environ["GAZEBO_MODEL_PATH"] = "/home/luisvviana/ros2_ws/src/bug1_turtle/worlds:/opt/ros/humble/share/turtlebot3_gazebo/models"

    # Comando para iniciar Gazebo com factory plugin
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Spawn do TurtleBot3 após um delay de 5s (garante que o serviço /spawn_entity esteja pronto)
    spawn_turtlebot = TimerAction(
        period=5.0,
        actions=[
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
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_turtlebot
    ])
