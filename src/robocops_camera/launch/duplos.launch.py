import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nn_name             = LaunchConfiguration('nn_name', default="yolo11n_3.blob")
    resource_base_folder = LaunchConfiguration('resource_base_folder', default=
                                               os.path.join(get_package_share_directory('robocops_camera'), 'resources'))
    queue_size = LaunchConfiguration('queue_size', default=30)
    
    declare_nn_name_cmd = DeclareLaunchArgument(
        'nn_name',
        default_value=nn_name,
        description='Path to the object detection blob needed for detection'
    )
    
    declare_resource_base_folder_cmd = DeclareLaunchArgument(
        'resource_base_folder',
        default_value=resource_base_folder,
        description='Path to the resources folder which contains the default blobs for the network'
    )

    # Node for object detection publisher
    duplo_detection_publisher = launch_ros.actions.Node(
        package='robocops_camera', executable='robocops_camera_duplos',
        output='screen',
        parameters=[{'nn_name': nn_name},
                    {'resource_base_folder': resource_base_folder},
                    {'queue_size': queue_size}]
    )

    # Launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_nn_name_cmd)
    ld.add_action(declare_resource_base_folder_cmd)
    
    ld.add_action(duplo_detection_publisher)

    return ld