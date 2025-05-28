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
    rgb_resolution_str   = LaunchConfiguration('rgb_resolution_str', default='720p')
    with_display         = LaunchConfiguration('with_display', default='false')
    with_processor         = LaunchConfiguration('with_processor', default='true')

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
    
    declare_with_processor_cmd = DeclareLaunchArgument(
        'with_processor',
        default_value=with_display,
        description='Enable or disable processor node of duplos'
    )

    declare_queue_size_cmd = DeclareLaunchArgument(
        'queue_size',
        default_value=queue_size,
        description='Enable or disable publishing of RGB and depth images along with detections'
    )

    # Node for object detection publisher
    duplo_detection_publisher = launch_ros.actions.Node(
        package='robocops_duplos', executable='duplo_publisher',
        output='screen',
        parameters=[{'nn_name': nn_name},
                    {'resource_base_folder': resource_base_folder},
                    {'rgb_resolution_str': rgb_resolution_str},
                    {'with_display': with_display},
                    {'queue_size': queue_size}]
    )

    # Node for detections display publisher
    duplo_detection_viewer = launch_ros.actions.Node(
        package='robocops_duplos', executable='duplo_viewer',
        output='screen',
        condition=IfCondition(with_display)
    )
    
    # Node for detections display publisher
    duplo_processor = launch_ros.actions.Node(
        package='robocops_duplos', executable='duplo_processor',
        output='screen',
        condition=IfCondition(with_processor)
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_nn_name_cmd)
    ld.add_action(declare_resource_base_folder_cmd)
    ld.add_action(declare_mono_resolution_cmd)
    ld.add_action(declare_with_display_cmd)
    ld.add_action(declare_queue_size_cmd)
    ld.add_action(declare_with_processor_cmd)
    
    ld.add_action(duplo_detection_publisher)
    ld.add_action(duplo_detection_viewer)
    ld.add_action(duplo_processor)

    return ld