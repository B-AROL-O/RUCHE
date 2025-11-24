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

from os import path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = 'ruche_ros2_control'
    # TODO: Add instruction of what to modify/add to add another robot

    ###################
    # Input arguments #
    ###################
    declared_arguments = []

    # Robot name
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="diffelegoo",
            description="name of the robot to control"
        )
    )

    # Simulation
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulate",
            default_value="false",
            description="true: launch the simulation, "
            "false: control the actual hardware"
        )
    )

    # Initializa arguments
    robot_name = LaunchConfiguration("robot_name")
    simulate = LaunchConfiguration("simulate")

    # Compute file names
    controllers_name = [robot_name, '_controllers.yaml']
    urdf_name = 'robot.urdf.xacro'

    #################
    # Node settings #
    #################

    # URDF robot description
    # interpret the xacro file
    # It will run the 'xacro xacro_file' command
    # It will point to the xacro file in the install folder!
    # (be consistent to setup.py)
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            "urdf",
            urdf_name
        ]),
        " robot_name:=",
        robot_name,
        " simulate:=",
        simulate,
    ])

    robot_description_params = {"robot_description": robot_description}

    # Controllers
    # Load the controller configurations
    # (Controller manager + motion controller + joint state broadcaster)
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "config",
        controllers_name
    ])

    # Gazebo world config file
    world_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "worlds",
        "empty.sdf"
    ])

    # Gazebo bridge config
    gz_bridge_params = path.join(
        get_package_share_directory(pkg_name),
        'config/gz_bridge.yaml')

    #########
    # NODES #
    #########

    # Start Gazebo
    gz_sim_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
                ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(simulate)
    )

    # Gazebo bridge
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        # arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gz_bridge_params}',
        ],
        condition=IfCondition(simulate)
    )

    # State publisher node
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        condition=UnlessCondition(simulate)
    )

    ############
    # SPAWNERS #
    ############

    # Spawners are one-shot nodes which putposes is to launch other nodes
    # We can use them to define a launch sequence using events
    # x_node is launched by x_node_spawner
    # on exit of x_node_spawner we launch y_node_spawner

    # The sequence we want to attain is:
    # joint_state_broadcaster -> robot_controller

    # Joint state broadcast spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ],
    )

    # Robot controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "base_controller",
            "--param-file", robot_controllers,
            # Optionally redirect /cmd_vel to the controller
            # "--controller-ros-args",
            # "-r /base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    # Spawn robot in Gazebo Harmonic
    robot_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "robot",
            "-z", "0.2"
        ],
        condition=IfCondition(simulate)
    )

    #############
    # SEQUENCES #
    #############

    # controller sequence
    seq_robot_controller_after_joint_st_br = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner]
        )
    )

    # Run the nodes and launch arguments
    nodes = [
        gz_sim_launchfile,
        ros_gz_bridge_node,
        robot_spawner,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        seq_robot_controller_after_joint_st_br,
    ]

    return LaunchDescription(declared_arguments + nodes)
