import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_bt = DeclareLaunchArgument('bt', default_value='robocops_tree.xml')

    xml_file_path = os.path.join(
        get_package_share_directory("robocops_brain"),
        "behavior_trees",
        "test_slopeup.xml"
        # "test_is_door_open_tree.xml"
    )
    
    locations_file_path = os.path.join(
        get_package_share_directory("robocops_brain"),
        "config",
        "map_locations.yaml"
    )
    
    bt_runner_node = Node(
        package='robocops_brain',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        parameters=[{
            'tree_xml_file': xml_file_path,
            'locations_file': locations_file_path
        }]
    )

    ld = LaunchDescription()
    ld.add_action(bt_runner_node)

    return ld
