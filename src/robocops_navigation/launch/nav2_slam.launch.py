from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
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
    urdf_file_path = PathJoinSubstitution([
        get_package_share_directory('robocops_gazebo'),
        'description',
        PythonExpression(["'", LaunchConfiguration('robot_model'), "' + '.urdf.xacro'"])
    ])

    gazebo_params_file = os.path.join(get_package_share_directory('robocops_gazebo'), 'config', 'gz_bridge.yaml')
    rviz_params_file = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'rviz2_nav2_params.rviz')
    nav2_yaml = os.path.join(get_package_share_directory('robocops_navigation'), 'config', 'nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('robocops_navigation'), 'maps', 'test_arena.yaml')
    remappings_nav = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

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
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_file_path, ' sim_mode:=' ])
        }],
        remappings=remappings_nav,
    )

    ld.add_action(robot_state_publisher_cmd)
    # ------

    # --- include RVIZ2 launch file and setup ---
    rviz_visualise_robot = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_params_file],
        output='screen'
    )
    ld.add_action(rviz_visualise_robot)
    # ------

    # --- include NAV2 launch file and setup ---
    nav2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch/bringup_launch.py')
        ),
        launch_arguments={'slam': 'False', 'map': map_file, 'use_sim_time': 'True', 'params_file': nav2_yaml, 'autostart': 'True'}.items()
    )

    ld.add_action(nav2_launch_file)
    # ------

    return ld