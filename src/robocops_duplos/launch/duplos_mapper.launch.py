from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robocops_duplos',
            executable='duplos_mapper',
            name='duplos_mapper',
            output='screen',
        ),
    ])
