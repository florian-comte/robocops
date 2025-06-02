import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    ld = LaunchDescription()

    # declare important parameters
    rviz_params_file = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'rviz2_view_lidar.rviz')

    # --- launch the RPLIDAR node ---
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )
    ld.add_action(lidar_node)
    # ------

    # --- launch RVIZ2 configuration ---
    rviz_visualise_robot = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_params_file],
        output='screen'
    )
    ld.add_action(rviz_visualise_robot)
    # ------
 
    return ld