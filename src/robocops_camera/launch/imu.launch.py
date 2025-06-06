import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robocops_duplos',   
            executable='robocops_imu', 
            name='robocops_imu', 
            output='screen',           
                   )
    ])
