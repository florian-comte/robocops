import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Node for detections display publisher
    detections_display_publisher = launch_ros.actions.Node(
        package='robocops_camera', executable='duplo_detection_viewer',
        output='screen',
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(detections_display_publisher)

    return ld
