from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_bt_file = os.path.join(
        get_package_share_directory('robocops_brain'),
        'behavior_trees',
        'robocops_tree.xml'
    )

    return LaunchDescription([
        Node(
            package='robocops_brain',
            executable='BT',
            name='robocops_bt',
            output='screen',
            parameters=[{
                'bt_path': default_bt_file
            }]
        )
    ])
