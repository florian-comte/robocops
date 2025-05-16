from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression

def generate_launch_description():
    robot_model = LaunchConfiguration('robot_model')

    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='robocops_2_wheels',
        description='Robot model to simulate')

    urdf_file_path = PathJoinSubstitution([
        get_package_share_directory('robocops_gazebo'),
        'description',
        PythonExpression(["'", robot_model, "' + '.urdf.xacro'"])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_file_path, ' sim_mode:=' ])
        }],
    )

    return LaunchDescription([
        declare_robot_model,
        robot_state_publisher_node
    ])