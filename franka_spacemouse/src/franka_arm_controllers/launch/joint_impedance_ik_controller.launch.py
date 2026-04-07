#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "on")
    return bool(value)


def parse_optional_bool(value):
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("auto", "yaml", "config", ""):
            return None
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off"):
            return False
    raise ValueError(
        f"Expected optional bool value (auto/true/false), got: {value!r}"
    )


def generate_robot_nodes(context):
    config_file_name = LaunchConfiguration("robot_config_file").perform(context)
    launch_start_inactive_override = parse_optional_bool(
        LaunchConfiguration("start_controller_inactive").perform(context)
    )
    launch_spawn_franka_state_broadcaster_override = parse_optional_bool(
        LaunchConfiguration("spawn_franka_robot_state_broadcaster").perform(context)
    )
    package_config_dir = FindPackageShare("franka_arm_controllers").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []
    for item_name, config in configs.items():
        namespace = config["namespace"]
        config_start_inactive = to_bool(config.get("start_controller_inactive", False))
        start_controller_inactive = (
            launch_start_inactive_override
            if launch_start_inactive_override is not None
            else config_start_inactive
        )
        config_spawn_franka_state_broadcaster = to_bool(
            config.get("spawn_franka_robot_state_broadcaster", False)
        )
        spawn_franka_state_broadcaster = (
            launch_spawn_franka_state_broadcaster_override
            if launch_spawn_franka_state_broadcaster_override is not None
            else config_spawn_franka_state_broadcaster
        )
        spawner_arguments = [
            "joint_impedance_ik_controller",
            "--controller-manager-timeout",
            "30",
        ]
        if start_controller_inactive:
            spawner_arguments.append("--inactive")
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("franka_arm_controllers"), "launch", "franka.launch.py"]
                    )
                ),
                launch_arguments={
                    "arm_id": str(config["arm_id"]),
                    "arm_prefix": str(config["arm_prefix"]),
                    "namespace": str(namespace),
                    "urdf_file": str(config["urdf_file"]),
                    "robot_ip": str(config["robot_ip"]),
                    "load_gripper": str(config["load_gripper"]),
                    "use_fake_hardware": str(config["use_fake_hardware"]),
                    "fake_sensor_commands": str(config["fake_sensor_commands"]),
                    "joint_sources": ",".join(config["joint_sources"]),
                    "joint_state_rate": str(config["joint_state_rate"]),
                    "arm_mounting_orientation": str(config["arm_mounting_orientation"]),
                    "spawn_franka_robot_state_broadcaster": str(
                        spawn_franka_state_broadcaster
                    ),
                }.items(),
            )
        )
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=spawner_arguments,
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_arm_controllers"),
                            "config",
                            "controllers.yaml",
                        ]
                    )
                ],
                output="screen",
            )
        )
    if any(str(config.get("use_rviz", "false")).lower() == "true" for config in configs.values()):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "--display-config",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_description"),
                            "rviz",
                            "visualize_franka_duo.rviz",
                        ]
                    ),
                ],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config_file",
                default_value="example_fr3_config.yaml",
                description="Name of the robot configuration file to load (relative to config/ in franka_arm_controllers)",
            ),
            DeclareLaunchArgument(
                "start_controller_inactive",
                default_value="auto",
                description="Controller startup mode: auto(use YAML), true(inactive), false(active).",
            ),
            DeclareLaunchArgument(
                "spawn_franka_robot_state_broadcaster",
                default_value="auto",
                description="Franka state broadcaster startup: auto(use YAML), true, false.",
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
