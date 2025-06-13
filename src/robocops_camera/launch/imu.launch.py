from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robocops_camera',
            executable='robocops_camera_imu',
            name='robocops_camera_imu',
            parameters=[
                {'imu_mode': 1},
                {'linear_accel_covariance': 0.02},
                {'angular_vel_covariance': 0.0},
                {'enable_ros_base_time_update': True}
            ],
            output='screen'
        )
    ])
