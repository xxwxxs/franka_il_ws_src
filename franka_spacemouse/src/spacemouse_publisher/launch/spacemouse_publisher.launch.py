import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
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


def to_bool_list(value, default):
    if value is None:
        return default
    if isinstance(value, (list, tuple)):
        return [to_bool(v) for v in value]
    return default


def generate_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    package_config_dir = FindPackageShare("spacemouse_publisher").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []
    for item_name, config in configs.items():
        nodes.append(
            Node(
                package="spacemouse_publisher",
                executable="pyspacemouse_publisher",
                name="spacemouse_publisher",
                namespace=str(config["namespace"]),
                output="screen",
                parameters=[
                    {"operator_position_front": to_bool(config["operator_position_front"])},
                    {"device_path": str(config["device_path"])},
                    {"auto_reconnect": to_bool(config.get("auto_reconnect", True))},
                    {"reconnect_interval_sec": float(config.get("reconnect_interval_sec", 1.0))},
                    {"enable_deadman": to_bool(config.get("enable_deadman", False))},
                    {"deadman_button_index": int(config.get("deadman_button_index", 1))},
                    {"linear_sensitivity": float(config.get("linear_sensitivity", 1.0))},
                    {"angular_sensitivity": float(config.get("angular_sensitivity", 1.0))},
                    {"input_deadzone": float(config.get("input_deadzone", 0.0))},
                    {
                        "intent_decoupling_enabled": to_bool(
                            config.get("intent_decoupling_enabled", True)
                        )
                    },
                    {"intent_decoupling_ratio": float(config.get("intent_decoupling_ratio", 1.8))},
                    {
                        "cross_intent_attenuation": float(
                            config.get("cross_intent_attenuation", 0.20)
                        )
                    },
                    {
                        "z_pitch_decoupling_enabled": to_bool(
                            config.get("z_pitch_decoupling_enabled", True)
                        )
                    },
                    {"z_pitch_dominance_ratio": float(config.get("z_pitch_dominance_ratio", 1.25))},
                    {"z_pitch_attenuation": float(config.get("z_pitch_attenuation", 0.15))},
                    {"command_mode": str(config.get("command_mode", "full"))},
                    {
                        "axis_mapping_linear": list(
                            config.get("axis_mapping_linear", ["-y", "x", "z"])
                        )
                    },
                    {
                        "axis_mapping_angular": list(
                            config.get(
                                "axis_mapping_angular", ["-roll", "-pitch", "-yaw"]
                            )
                        )
                    },
                    {
                        "axis_inversion_linear": to_bool_list(
                            config.get("axis_inversion_linear"), [False, False, False]
                        )
                    },
                    {
                        "axis_inversion_angular": to_bool_list(
                            config.get("axis_inversion_angular"), [False, False, False]
                        )
                    },
                    {
                        "debug_axis_calibration": to_bool(
                            config.get("debug_axis_calibration", False)
                        )
                    },
                    {
                        "debug_log_interval_sec": float(
                            config.get("debug_log_interval_sec", 0.5)
                        )
                    },
                    {
                        "enable_gripper_buttons": to_bool(
                            config.get("enable_gripper_buttons", False)
                        )
                    },
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="example_fr3_config.yaml",
                description="Name of the spacemouse configuration file to load",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )
