from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    rviz_params_file = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'rviz2_nav2_params.rviz')

    # --- launch RVIZ2 config visualisation file ---
    rviz_robocops_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_params_file],
        output='screen'
    )

    ld.add_action(rviz_robocops_cmd)
    # ------

    return ld