# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    gpio_configuration = LaunchConfiguration("gpio_configuration")
    use_mock = LaunchConfiguration("use_mock")

    nodes_to_launch = []

    # Get parameters for the Servo node
    servo_yaml = load_yaml("fanuc_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Conditionally include the appropriate control launch file
    include_fanuc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_forward_command"),
                    "launch",
                    "fanuc_forward_command.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "robot_ip": robot_ip,
            "gpio_configuration": gpio_configuration,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "true",
            "use_mock": use_mock,
        }.items(),
        condition=UnlessCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_control)

    include_fanuc_mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_mock_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "gpio_configuration": gpio_configuration,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "true",
        }.items(),
        condition=IfCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_mock_control)

    description_arguments = {
        "robot_ip": robot_ip.perform(context),
        "use_mock": use_mock.perform(context),
        "gpio_configuration": gpio_configuration.perform(context),
    }

    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot",
        f"{robot_model.perform(context)}.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_model.perform(context), package_name="fanuc_moveit_config"
        )
        .robot_description(file_path=urdf_full_path, mappings=description_arguments)
        .robot_description_semantic(
            file_path=f"srdf/{robot_model.perform(context)}.srdf"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict()],
    )
    nodes_to_launch.append(move_group_node)

    # launch servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
        output="screen",
    )
    nodes_to_launch.append(servo_node)

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model (required).",
            choices=[
                "crx3ia",
                "crx5ia",
                "crx10ia",
                "crx10ia_l",
                "crx20ia_l",
                "crx30ia",
            ],
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="The robot's IP address.",
        ),
        DeclareLaunchArgument(
            "ros2_control_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "ros2_controllers.yaml",
                ]
            ),
            description="ROS 2 control configuration file the controllers.",
        ),
        DeclareLaunchArgument(
            "gpio_configuration",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "example_gpio_config.yaml",
                ]
            ),
            description="YAML file configuration to specify the GPIO configuration..",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Whether to use a mock hardware interface.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
