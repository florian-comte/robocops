from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    nav2_yaml = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'nav2_params_DWB.yaml')
    map_file = os.path.join(get_package_share_directory('robocops_navigation'), 'maps', 'final_arena_blank.yaml')

    nav2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch/bringup_launch.py')
        ),
        launch_arguments={'slam': 'False', 'map': map_file, 'use_sim_time': 'False', 'params_file': nav2_yaml, 'autostart': 'True'}.items()
    )


    ld.add_action(nav2_launch_file)

    return ld