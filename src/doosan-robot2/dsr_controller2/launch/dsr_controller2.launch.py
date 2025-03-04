# Copyright 2023 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'model',
        default_value='a0509',
        description='Robot Model'
    ),
    DeclareLaunchArgument(
        'color',
        default_value='blue',
        description='Robot Color'
    ),
    DeclareLaunchArgument('name', default_value='', description='NAME_SPACE'),
]

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("dsr_description2"),
            "xacro",
            "a0509.urdf.xacro",
        ]),
    ])

    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("dsr_controller2"),
        "config",
        "dsr_controller2.yaml",
    ])

    # Single instance of r_controller
    r_controller_node = Node(
        package='hum_pkg',
        executable='r_controller',
        namespace=LaunchConfiguration('name'),
        output='screen',
        parameters=[robot_description]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration('name'),
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "/controller_manager"],
    )

    # Delay controller spawning after joint state broadcaster
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        r_controller_node,  # Add single instance here
    ]

    return LaunchDescription(ARGUMENTS + nodes)