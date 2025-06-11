from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    ld = LaunchDescription()

    nav2_yaml = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'nav2_params_MPPI_keepout.yaml')
    map_file = os.path.join(get_package_share_directory('robocops_navigation'), 'maps', 'final_arena_blank.yaml')
    keepout_map_file = os.path.join(get_package_share_directory('robocops_navigation'), 'maps', 'final_arena_keepout.yaml')
    keepout_params_file = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'keepout_params.yaml')

    nav2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch/bringup_launch.py')
        ),
        launch_arguments={'slam': 'False', 'map': map_file, 'use_sim_time': 'False', 'params_file': nav2_yaml, 'autostart': 'True'}.items()
    )

    # Make re-written yaml
    param_substitutions = {
        'use_sim_time': False,
        'yaml_filename': keepout_map_file}

    configured_params = RewrittenYaml(
        source_file=keepout_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    start_lifecycle_manager_cmd = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_costmap_filters',
    output='screen',
    emulate_tty=True,
    parameters=[{'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}])

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[configured_params]
    )

    start_costmap_filter_info_server_cmd = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[configured_params])


    ld.add_action(nav2_launch_file)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)

    return ld