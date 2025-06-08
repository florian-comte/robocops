from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths to each package
    control_pkg = get_package_share_directory('robocops_control')
    camera_pkg = get_package_share_directory('robocops_camera')
    nav_pkg = get_package_share_directory('robocops_navigation')
    
    imu_node = Node(
        package="ros2_icm20948",
        executable="icm20948_node",
        name="icm20948_node",
        parameters=[
            {"i2c_address": 0x69},
            {"frame_id": "base_link"},
            {"pub_rate": 50},
        ],
    )
    
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(nav_pkg, 'config', 'imu_filter.yaml')],
    )

    # EKF Node (robot_localization)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node_filter',
        output='screen',
        parameters=[os.path.join(nav_pkg, 'config', 'ekf.yaml')],
    )

    # Launch other subsystems
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'diffbot.launch.py')
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg, 'launch', 'duplos.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'view_lidar.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'start_nav.launch.py')
        )
    )

    # Delay Nav2 to ensure other systems are initialized
    delayed_nav2 = TimerAction(
        period=10.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        imu_node,
        imu_filter_node,
        robot_localization_node,
        diffbot_launch,
        camera_launch,
        lidar_launch,
        delayed_nav2
    ])
