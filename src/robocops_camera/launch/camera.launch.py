import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths to the launch files
    robocops_camera_dir = get_package_share_directory('robocops_camera')
    duplos_launch_path = os.path.join(robocops_camera_dir, 'launch', 'duplos.launch.py')
    imu_launch_path = os.path.join(robocops_camera_dir, 'launch', 'imu.launch.py')

    # Declare shared arguments if needed (optional)
    nn_name_arg = DeclareLaunchArgument(
        'nn_name',
        default_value='yolo11n_3.blob',
        description='Neural network blob filename'
    )

    resource_base_folder_arg = DeclareLaunchArgument(
        'resource_base_folder',
        default_value=os.path.join(robocops_camera_dir, 'resources'),
        description='Base folder for NN resources'
    )

    # Include duplo detection launch
    duplos_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(duplos_launch_path),
        launch_arguments={
            'nn_name': LaunchConfiguration('nn_name'),
            'resource_base_folder': LaunchConfiguration('resource_base_folder')
        }.items()
    )

    # Include IMU launch
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_path)
    )

    return LaunchDescription([
        nn_name_arg,
        resource_base_folder_arg,
        duplos_launch,
        imu_launch
    ])
