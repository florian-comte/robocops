from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- launch the DIFFBOT file ---
    launch_robocops_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robocops_control'),
                         'bringup/launch/diffbot.launch.py')
        ),
    )

    ld.add_action(launch_robocops_cmd)
    # ------

    # --- launch LIDAR FILTERED file ---
    launch_lidar_filtered_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robocops_navigation'),
                         'launch/view_lidar.launch.py')
        ),
    )

    ld.add_action(launch_lidar_filtered_cmd)
    # ------

    # --- launch NAV2 STACK file ---
    launch_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robocops_navigation'),
                         'launch/start_nav.launch.py')
        ),
    )

    ld.add_action(launch_nav2_cmd)
    # ------

    return ld