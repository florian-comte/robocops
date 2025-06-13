# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch.substitutions import LaunchConfiguration, Command

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import os

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            name='simulation',
            default_value='false',
            description='Use simulation (Gazebo)'
        )
    ]
    
    xacro_file = os.path.join(get_package_share_directory('robocops_description'),'description', 'urdf', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' simulation:=', LaunchConfiguration('simulation')])

    params = {'robot_description': robot_description_config, 'use_sim_time': LaunchConfiguration('simulation')}

    return LaunchDescription(declared_arguments + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
        ),
    ])