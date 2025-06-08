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

    # EKF Node (robot_localization)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(nav_pkg, 'config', 'ekf.yaml')],


    twist_mux_params = os.path.join(nav_pkg,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel')]
        )
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
        # robot_localization_node,
        diffbot_launch,
        camera_launch,
        lidar_launch,
        delayed_nav2
    ])
