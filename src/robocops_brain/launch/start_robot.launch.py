import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # xml_file_name = LaunchConfiguration('xml_file_name', default='robocops_tree.xml')

    # declare_xml_file_name = DeclareLaunchArgument(
    #     'xml_file_name',
    #     default_value='robocops_tree.xml',
    #     description='Name of the behavior tree XML file to execute'
    # )

    xml_file_path = os.path.join(
        get_package_share_directory("robocops_brain"),
        "behavior_trees",
        "robocops_tree.xml"
    )
    
    print(xml_file_path)

    bt_runner_node = Node(
        package='robocops_brain',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        parameters=[{
            'tree_xml_file': xml_file_path
        }]
    )

    ld = LaunchDescription()
    # ld.add_action(declare_xml_file_name)
    ld.add_action(bt_runner_node)

    return ld
