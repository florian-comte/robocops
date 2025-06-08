from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robocops_description"), "description", "urdf", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robocops_control"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("robocops_control"), "rviz", "diffbot.rviz"]
    # )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    joint_state_publiser_node =  Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[robot_description],
    )

    # rviz = LaunchConfiguration("rviz")

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(rviz)
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file", robot_controllers,
            "--controller-ros-args", "--ros-args --remap /diffbot_base_controller/odom:=/odom"
        ],
    )


    

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--param-file", robot_controllers],
    )

    delay_gpio_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gpio_controller_spawner],
        )
    )

    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_publiser_node,
        robot_controller_spawner,
        delay_gpio_after_robot_controller_spawner,
        #delay_rviz_after_joint_state_broadcaster_spawner,
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)