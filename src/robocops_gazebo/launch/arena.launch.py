import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, PathJoinSubstitution

def generate_launch_description():
    robot_model = LaunchConfiguration('robot_model')
    world_input = LaunchConfiguration('world')
    initial_x_pose = LaunchConfiguration('initial_x_pose')
    initial_y_pose = LaunchConfiguration('initial_y_pose')

    declare_world_input_cmd = DeclareLaunchArgument(
        'world',
        default_value='arena',
        description='World file name without extension')
    
    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='robocops_2_wheels',
        description='Model of the robot')
    
    declare_initial_x_pose_cmd = DeclareLaunchArgument(
        'initial_x_pose',
        default_value='-8.5',
        description='Initial X pose of the robot')
    
    declare_initial_y_pose_cmd = DeclareLaunchArgument(
        'initial_y_pose',
        default_value='-2',
        description='Initial Y pose of the robot')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.join(get_package_share_directory('robocops_gazebo'), 'launch'), 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'robot_model': robot_model}.items()
    )
    
    world = PathJoinSubstitution([
        get_package_share_directory('robocops_gazebo'),
        'worlds',
        PythonExpression(["'", world_input, "' + '.world'"])
    ])

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                    '-name', 'robot',
                                    '-x', initial_x_pose,
                                    '-y', initial_y_pose,
                                    '-z', '0.01' 
                                    ],
                        output='screen')

    bridge_params = os.path.join(get_package_share_directory("robocops_gazebo"),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription([
        declare_world_input_cmd,
        declare_robot_model_cmd,
        declare_initial_x_pose_cmd,
        declare_initial_y_pose_cmd,
        robot_state_publisher_cmd,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
    ])
