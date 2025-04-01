import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    nn_name             = LaunchConfiguration('nn_name', default="yolo11n_3.blob")
    resource_base_folder = LaunchConfiguration('resource_base_folder', default=
                                               os.path.join(get_package_share_directory('robocops_camera'), 'resources'))
    rgb_resolution_str   = LaunchConfiguration('rgb_resolution_str', default='1080p')
    with_display         = LaunchConfiguration('with_display', default='false')

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

    declare_mono_resolution_cmd = DeclareLaunchArgument(
        'rgb_resolution_str',
        default_value=rgb_resolution_str,
        description='Contains the resolution of the Color Camera'
    )

    declare_with_display_cmd = DeclareLaunchArgument(
        'with_display',
        default_value=with_display,
        description='Enable or disable publishing of RGB and depth images along with detections'
    )

    # Node for object detection publisher
    duplo_detection_publisher = launch_ros.actions.Node(
        package='robocops_camera', executable='duplo_detection_publisher',
        output='screen',
        parameters=[{'nn_name': nn_name},
                    {'resource_base_folder': resource_base_folder},
                    {'rgb_resolution_str': rgb_resolution_str},
                    {'with_display': with_display}]  # Pass with_display parameter
    )

    # Node for detections display publisher
    detections_display_publisher = launch_ros.actions.Node(
        package='robocops_camera', executable='detections_display_publisher',
        output='screen',
        condition=IfCondition(with_display)
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_nn_name_cmd)
    ld.add_action(declare_resource_base_folder_cmd)
    ld.add_action(declare_mono_resolution_cmd)
    ld.add_action(declare_with_display_cmd)
    
    ld.add_action(duplo_detection_publisher)
    ld.add_action(detections_display_publisher)

    return ld
