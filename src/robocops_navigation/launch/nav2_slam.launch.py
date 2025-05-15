from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- declare args for launch file ---
    declare_world = DeclareLaunchArgument('world', default_value='arena')
    declare_robot = DeclareLaunchArgument('robot_model', default_value='robocops_2_wheels')
    declare_init_pose_x = DeclareLaunchArgument('initial_pose_x', default_value='-8.5')
    declare_init_pose_y = DeclareLaunchArgument('initial_pose_y', default_value='-2')

    gazebo_world_file = PathJoinSubstitution([
        get_package_share_directory('robocops_gazebo'),
        'worlds',
        PythonExpression(["'", LaunchConfiguration('world'), "' + '.world'"])
    ])
    gazebo_params_file = os.path.join(get_package_share_directory('robocops_gazebo'), 'config', 'gz_bridge.yaml')
    rviz_params_file = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'rviz2_nav2_params.rviz')
    nav2_yaml = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('robocops_navigation'), 'map', 'test_arena.yaml')
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    ld.add_action(declare_world)
    ld.add_action(declare_robot)
    ld.add_action(declare_init_pose_x)
    ld.add_action(declare_init_pose_y)
    # ------

    # --- include GAZEBO launch file and setup ---
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch/gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 ', gazebo_world_file], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                    '-name', 'robot',
                                    '-x', LaunchConfiguration('initial_pose_x'),
                                    '-y', LaunchConfiguration('initial_pose_y'),
                                    '-z', '0.01' 
                                    ],
                        output='screen'
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gazebo_params_file}',
        ]
    )

    ld.add_action(gazebo_launch_file)
    ld.add_action(spawn_entity)
    ld.add_action(ros_gz_bridge)
    # ------

    # --- include ROBOT STATE PUBLISHER launch file and setup ---
    robot_state_publisher_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robocops_gazebo'), 
                         'launch/robot_state_publisher.launch.py')
        ),
        launch_arguments={'robot_model': LaunchConfiguration('robot_model')}.items()
    )

    ld.add_action(robot_state_publisher_launch_file)
    # ------

    # --- include RVIZ2 launch file and setup ---
    rviz_visualise_robot = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_params_file],
        output='screen'
        #parameters=[{'use_sim_time': True}]
    )
    ld.add_action(rviz_visualise_robot)
    # ------

    # --- include NAV2 launch file and setup ---
    lifecycle_nodes = ['map_server', 
                       'amcl',
                       'planner_server',
                       'controller_server',
                       'recoveries_server',
                       'bt_navigator'
                       ]
    #remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped'),('/odom','/odom_filtered')] #odom_filtered for sensor fusion only
    #remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]

    nav2_map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}])

    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml])
                
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_yaml])#,
        #remappings=remappings)

    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml])
        
    #nav2_recoveries = Node(
    #    package='nav2_recoveries',
    #    executable='recoveries_server',
    #   name='recoveries_server',
    #    parameters=[nav2_yaml],
    #    output='screen')

    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_yaml])
        
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])
    
    ld.add_action(nav2_map_server)
    ld.add_action(nav2_amcl)
    ld.add_action(nav2_controller)
    ld.add_action(nav2_planner)
    #ld.add_action(nav2_recoveries)
    ld.add_action(nav2_bt_navigator)
    ld.add_action(nav2_lifecycle_manager)
    # ------

    return ld