import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import pyspacemouse


class SpaceMousePublisher(Node):
    """
    A ROS2 Node that publishes 3D mouse input as Twist messages.

    This class initializes a ROS2 publisher to publish geometry_msgs/Twist messages
    based on the input from a 3D SpaceMouse device. It uses the pyspacemouse library
    to read the device state and publishes the corresponding linear and angular
    velocities at a fixed rate.
    """

    def __init__(self):
        super().__init__("spacemouse_publisher")
        self.get_logger().info("Initializing SpaceMouse publisher...")

        self.declare_parameter("operator_position_front", True)
        self._operator_position_front = (
            self.get_parameter("operator_position_front").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Operator position front: {self._operator_position_front}")

        self.declare_parameter("device_path", "")
        self._device_path = self.get_parameter("device_path").get_parameter_value().string_value
        self.declare_parameter("auto_reconnect", True)
        self._auto_reconnect = (
            self.get_parameter("auto_reconnect").get_parameter_value().bool_value
        )
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self._reconnect_interval_sec = (
            self.get_parameter("reconnect_interval_sec").get_parameter_value().double_value
        )
        if self._reconnect_interval_sec <= 0.0:
            self.get_logger().warning("reconnect_interval_sec <= 0.0, falling back to 1.0s.")
            self._reconnect_interval_sec = 1.0
        self.declare_parameter("enable_deadman", False)
        self._enable_deadman = (
            self.get_parameter("enable_deadman").get_parameter_value().bool_value
        )
        self.declare_parameter("deadman_button_index", 1)
        self._deadman_button_index = (
            self.get_parameter("deadman_button_index").get_parameter_value().integer_value
        )
        self.declare_parameter("linear_sensitivity", 1.0)
        self._linear_sensitivity = (
            self.get_parameter("linear_sensitivity").get_parameter_value().double_value
        )
        self.declare_parameter("angular_sensitivity", 1.0)
        self._angular_sensitivity = (
            self.get_parameter("angular_sensitivity").get_parameter_value().double_value
        )
        self.declare_parameter("input_deadzone", 0.0)
        self._input_deadzone = (
            self.get_parameter("input_deadzone").get_parameter_value().double_value
        )
        self.declare_parameter("intent_decoupling_enabled", True)
        self._intent_decoupling_enabled = (
            self.get_parameter("intent_decoupling_enabled").get_parameter_value().bool_value
        )
        self.declare_parameter("intent_decoupling_ratio", 1.8)
        self._intent_decoupling_ratio = (
            self.get_parameter("intent_decoupling_ratio").get_parameter_value().double_value
        )
        self.declare_parameter("cross_intent_attenuation", 0.20)
        self._cross_intent_attenuation = (
            self.get_parameter("cross_intent_attenuation").get_parameter_value().double_value
        )
        self.declare_parameter("z_pitch_decoupling_enabled", True)
        self._z_pitch_decoupling_enabled = (
            self.get_parameter("z_pitch_decoupling_enabled").get_parameter_value().bool_value
        )
        self.declare_parameter("z_pitch_dominance_ratio", 1.25)
        self._z_pitch_dominance_ratio = (
            self.get_parameter("z_pitch_dominance_ratio").get_parameter_value().double_value
        )
        self.declare_parameter("z_pitch_attenuation", 0.15)
        self._z_pitch_attenuation = (
            self.get_parameter("z_pitch_attenuation").get_parameter_value().double_value
        )
        self.declare_parameter("command_mode", "full")
        self._command_mode = (
            self.get_parameter("command_mode").get_parameter_value().string_value.lower()
        )
        self.declare_parameter("axis_mapping_linear", ["-y", "x", "z"])
        self._axis_mapping_linear = list(
            self.get_parameter("axis_mapping_linear")
            .get_parameter_value()
            .string_array_value
        )
        self.declare_parameter("axis_mapping_angular", ["-roll", "-pitch", "-yaw"])
        self._axis_mapping_angular = list(
            self.get_parameter("axis_mapping_angular")
            .get_parameter_value()
            .string_array_value
        )
        self.declare_parameter("axis_inversion_linear", [False, False, False])
        self._axis_inversion_linear = list(
            self.get_parameter("axis_inversion_linear")
            .get_parameter_value()
            .bool_array_value
        )
        self.declare_parameter("axis_inversion_angular", [False, False, False])
        self._axis_inversion_angular = list(
            self.get_parameter("axis_inversion_angular")
            .get_parameter_value()
            .bool_array_value
        )
        self.declare_parameter("debug_axis_calibration", False)
        self._debug_axis_calibration = (
            self.get_parameter("debug_axis_calibration").get_parameter_value().bool_value
        )
        self.declare_parameter("debug_log_interval_sec", 0.5)
        self._debug_log_interval_sec = (
            self.get_parameter("debug_log_interval_sec").get_parameter_value().double_value
        )
        self.declare_parameter("enable_gripper_buttons", False)
        self._enable_gripper_buttons = (
            self.get_parameter("enable_gripper_buttons")
            .get_parameter_value()
            .bool_value
        )
        if self._deadman_button_index < 0:
            self.get_logger().warning("deadman_button_index < 0, falling back to 0.")
            self._deadman_button_index = 0
        self._linear_sensitivity = max(0.0, float(self._linear_sensitivity))
        self._angular_sensitivity = max(0.0, float(self._angular_sensitivity))
        self._input_deadzone = min(0.4, max(0.0, float(self._input_deadzone)))
        self._intent_decoupling_ratio = max(1.0, float(self._intent_decoupling_ratio))
        self._cross_intent_attenuation = min(1.0, max(0.0, float(self._cross_intent_attenuation)))
        self._z_pitch_dominance_ratio = max(1.0, float(self._z_pitch_dominance_ratio))
        self._z_pitch_attenuation = min(1.0, max(0.0, float(self._z_pitch_attenuation)))
        if self._debug_log_interval_sec <= 0.0:
            self.get_logger().warning(
                "debug_log_interval_sec <= 0.0, falling back to 0.5s."
            )
            self._debug_log_interval_sec = 0.5
        if self._command_mode not in ("full", "translation_only", "rotation_only"):
            self.get_logger().warning(
                f"Unsupported command_mode '{self._command_mode}', falling back to 'full'."
            )
            self._command_mode = "full"
        self._axis_mapping_linear = self._normalize_axis_mapping(
            self._axis_mapping_linear, ["-y", "x", "z"], "axis_mapping_linear"
        )
        self._axis_mapping_angular = self._normalize_axis_mapping(
            self._axis_mapping_angular, ["-roll", "-pitch", "-yaw"], "axis_mapping_angular"
        )
        self._axis_inversion_linear = self._normalize_axis_inversion(
            self._axis_inversion_linear, "axis_inversion_linear"
        )
        self._axis_inversion_angular = self._normalize_axis_inversion(
            self._axis_inversion_angular, "axis_inversion_angular"
        )
        self.get_logger().info(
            f"auto_reconnect={self._auto_reconnect}, "
            f"reconnect_interval_sec={self._reconnect_interval_sec:.2f}, "
            f"enable_deadman={self._enable_deadman}, "
            f"deadman_button_index={self._deadman_button_index}, "
            f"linear_sensitivity={self._linear_sensitivity:.2f}, "
            f"angular_sensitivity={self._angular_sensitivity:.2f}, "
            f"input_deadzone={self._input_deadzone:.3f}, "
            f"intent_decoupling_enabled={self._intent_decoupling_enabled}, "
            f"intent_decoupling_ratio={self._intent_decoupling_ratio:.2f}, "
            f"cross_intent_attenuation={self._cross_intent_attenuation:.2f}, "
            f"z_pitch_decoupling_enabled={self._z_pitch_decoupling_enabled}, "
            f"z_pitch_dominance_ratio={self._z_pitch_dominance_ratio:.2f}, "
            f"z_pitch_attenuation={self._z_pitch_attenuation:.2f}, "
            f"command_mode={self._command_mode}, "
            f"axis_mapping_linear={self._axis_mapping_linear}, "
            f"axis_mapping_angular={self._axis_mapping_angular}, "
            f"axis_inversion_linear={self._axis_inversion_linear}, "
            f"axis_inversion_angular={self._axis_inversion_angular}, "
            f"debug_axis_calibration={self._debug_axis_calibration}, "
            f"debug_log_interval_sec={self._debug_log_interval_sec:.2f}, "
            f"enable_gripper_buttons={self._enable_gripper_buttons}"
        )

        self._twist_publisher = self.create_publisher(
            Twist, "franka_controller/target_cartesian_velocity_percent", 10
        )
        self._gripper_width_publisher = self.create_publisher(
            Float32, "gripper_client/target_gripper_width_percent", 10
        )
        self._timer = self.create_timer(0.01, self._timer_callback)
        self._last_open_attempt_time = self.get_clock().now()
        self._last_debug_log_time_sec = 0.0
        self._device_open_success = self._open_device()

    def _close_device(self):
        close_fn = getattr(pyspacemouse, "close", None)
        if callable(close_fn):
            try:
                close_fn()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Failed to close SpaceMouse handle cleanly: {exc}")

    def _open_device(self):
        callback_kwargs = dict(
            dof_callback=None,
            button_callback_arr=[
                pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
                pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            ],
        )
        candidate_paths = []
        if self._device_path:
            if not os.path.exists(self._device_path):
                self.get_logger().warning(
                    f"Configured device_path '{self._device_path}' does not exist. "
                    "Will fall back to auto discovery."
                )
            candidate_paths.append(self._device_path)
        candidate_paths.append("")
        candidate_paths = list(dict.fromkeys(candidate_paths))

        for candidate in candidate_paths:
            try:
                open_success = pyspacemouse.open(path=candidate, **callback_kwargs)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"Failed to open SpaceMouse at path '{candidate}': {exc}"
                )
                continue
            if open_success:
                if candidate:
                    self.get_logger().info(f"SpaceMouse opened at '{candidate}'.")
                else:
                    self.get_logger().info("SpaceMouse opened with auto device discovery.")
                return True

        try:
            hidraw_candidates = sorted(
                [f"/dev/{entry}" for entry in os.listdir("/dev") if entry.startswith("hidraw")]
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to enumerate /dev/hidraw*: {exc}")
            hidraw_candidates = []
        self.get_logger().error(
            f"Unable to open SpaceMouse. Configured path='{self._device_path}'. "
            f"Available hidraw devices: {hidraw_candidates}"
        )
        return False

    def _publish_zero_twist(self):
        zero_twist = Twist()
        self._twist_publisher.publish(zero_twist)

    @staticmethod
    def _sanitize_axis_value(value):
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(numeric_value):
            return 0.0
        return max(-1.0, min(1.0, numeric_value))

    def _apply_deadzone(self, value):
        if abs(value) < self._input_deadzone:
            return 0.0
        return value

    @staticmethod
    def _vector_norm(values):
        return math.sqrt(sum(v * v for v in values))

    @staticmethod
    def _resolve_mapped_axis(mapping_token, raw_axes):
        token = str(mapping_token).strip().lower()
        sign = -1.0 if token.startswith("-") else 1.0
        axis_name = token[1:] if token.startswith("-") else token
        if axis_name not in raw_axes:
            return 0.0
        return sign * raw_axes[axis_name]

    def _normalize_axis_mapping(self, mapping_values, defaults, parameter_name):
        allowed_axes = {"x", "y", "z", "roll", "pitch", "yaw"}
        if len(mapping_values) != 3:
            self.get_logger().warning(
                f"{parameter_name} must contain exactly 3 values, falling back to {defaults}."
            )
            return defaults
        normalized = []
        for value in mapping_values:
            token = str(value).strip().lower()
            axis_name = token[1:] if token.startswith("-") else token
            if axis_name not in allowed_axes:
                self.get_logger().warning(
                    f"Invalid axis token '{value}' in {parameter_name}, "
                    f"falling back to {defaults}."
                )
                return defaults
            normalized.append(token)
        return normalized

    def _normalize_axis_inversion(self, inversion_values, parameter_name):
        if len(inversion_values) != 3:
            self.get_logger().warning(
                f"{parameter_name} must contain exactly 3 values, falling back to [False, False, False]."
            )
            return [False, False, False]
        return [bool(v) for v in inversion_values]

    def _apply_axis_mapping_and_inversion(self, mapping_values, inversion_values, raw_axes):
        mapped_values = [self._resolve_mapped_axis(token, raw_axes) for token in mapping_values]
        for index, invert in enumerate(inversion_values):
            if invert:
                mapped_values[index] *= -1.0
        return mapped_values

    def _compute_se3_command(self, state):
        axis_x = self._sanitize_axis_value(getattr(state, "x", 0.0))
        axis_y = self._sanitize_axis_value(getattr(state, "y", 0.0))
        axis_z = self._sanitize_axis_value(getattr(state, "z", 0.0))
        axis_roll = self._sanitize_axis_value(getattr(state, "roll", 0.0))
        axis_pitch = self._sanitize_axis_value(getattr(state, "pitch", 0.0))
        axis_yaw = self._sanitize_axis_value(getattr(state, "yaw", 0.0))
        raw_axes = {
            "x": axis_x,
            "y": axis_y,
            "z": axis_z,
            "roll": axis_roll,
            "pitch": axis_pitch,
            "yaw": axis_yaw,
        }
        delta_pos = self._apply_axis_mapping_and_inversion(
            self._axis_mapping_linear, self._axis_inversion_linear, raw_axes
        )
        delta_rot = self._apply_axis_mapping_and_inversion(
            self._axis_mapping_angular, self._axis_inversion_angular, raw_axes
        )

        if not self._operator_position_front:
            delta_pos[0] *= -1.0
            delta_pos[1] *= -1.0
            delta_rot[0] *= -1.0
            delta_rot[1] *= -1.0

        delta_pos = [self._apply_deadzone(v) * self._linear_sensitivity for v in delta_pos]
        delta_rot = [self._apply_deadzone(v) * self._angular_sensitivity for v in delta_rot]

        if self._command_mode == "translation_only":
            delta_rot = [0.0, 0.0, 0.0]
        elif self._command_mode == "rotation_only":
            delta_pos = [0.0, 0.0, 0.0]

        if self._intent_decoupling_enabled:
            linear_norm = self._vector_norm(delta_pos)
            angular_norm = self._vector_norm(delta_rot)
            dominant_threshold = max(self._input_deadzone, 1.0e-6)
            if (
                linear_norm > dominant_threshold
                and linear_norm > self._intent_decoupling_ratio * angular_norm
            ):
                delta_rot = [v * self._cross_intent_attenuation for v in delta_rot]
            elif (
                angular_norm > dominant_threshold
                and angular_norm > self._intent_decoupling_ratio * linear_norm
            ):
                delta_pos = [v * self._cross_intent_attenuation for v in delta_pos]

        if self._z_pitch_decoupling_enabled:
            z_abs = abs(delta_pos[2])
            pitch_abs = abs(delta_rot[1])
            if z_abs > max(self._input_deadzone, 1.0e-6) and z_abs > (
                self._z_pitch_dominance_ratio * pitch_abs
            ):
                delta_rot[1] *= self._z_pitch_attenuation

        return delta_pos, delta_rot, raw_axes

    @staticmethod
    def _format_xyz(values):
        return f"{values[0]:+0.4f} {values[1]:+0.4f} {values[2]:+0.4f}"

    def _maybe_log_axis_calibration(self, raw_axes, delta_pos, delta_rot):
        if not self._debug_axis_calibration:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_debug_log_time_sec < self._debug_log_interval_sec:
            return
        self._last_debug_log_time_sec = now_sec
        raw_linear = [raw_axes["x"], raw_axes["y"], raw_axes["z"]]
        raw_angular = [raw_axes["roll"], raw_axes["pitch"], raw_axes["yaw"]]
        self.get_logger().info(
            "axis_calib raw_lin=[%s] raw_ang=[%s] mapped_lin=[%s] mapped_ang=[%s]"
            % (
                self._format_xyz(raw_linear),
                self._format_xyz(raw_angular),
                self._format_xyz(delta_pos),
                self._format_xyz(delta_rot),
            )
        )

    def _timer_callback(self):
        if not self._device_open_success:
            self._publish_zero_twist()
            if self._auto_reconnect:
                now = self.get_clock().now()
                if (
                    now - self._last_open_attempt_time
                ).nanoseconds / 1e9 >= self._reconnect_interval_sec:
                    self._last_open_attempt_time = now
                    self._device_open_success = self._open_device()
            return

        try:
            state = pyspacemouse.read()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"SpaceMouse read failed: {exc}")
            self._close_device()
            self._device_open_success = False
            self._publish_zero_twist()
            return
        if state is None:
            self._publish_zero_twist()
            return

        if self._enable_deadman:
            buttons = getattr(state, "buttons", None)
            deadman_pressed = False
            if isinstance(buttons, (list, tuple)) and len(buttons) > self._deadman_button_index:
                deadman_pressed = bool(buttons[self._deadman_button_index])
            if not deadman_pressed:
                self._publish_zero_twist()
                return

        delta_pos, delta_rot, raw_axes = self._compute_se3_command(state)
        self._maybe_log_axis_calibration(raw_axes, delta_pos, delta_rot)

        twist_msg = Twist()
        twist_msg.linear.x = delta_pos[0]
        twist_msg.linear.y = delta_pos[1]
        twist_msg.linear.z = delta_pos[2]
        twist_msg.angular.x = delta_rot[0]
        twist_msg.angular.y = delta_rot[1]
        twist_msg.angular.z = delta_rot[2]

        self._twist_publisher.publish(twist_msg)

    def _button_callback(self, state, buttons, pressed_buttons):
        if not self._enable_gripper_buttons:
            return

        target_gripper_width_percent_msg = Float32()
        if 0 in pressed_buttons:
            self.get_logger().info("Button 1 pressed")
            target_gripper_width_percent_msg.data = 0.0

        elif 1 in pressed_buttons:
            self.get_logger().info("Button 2 pressed")
            target_gripper_width_percent_msg.data = 1.0

        self._gripper_width_publisher.publish(target_gripper_width_percent_msg)

    def destroy_node(self):
        self._publish_zero_twist()
        self._close_device()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = SpaceMousePublisher()
    try:
        rclpy.spin(spacemouse_publisher)
    finally:
        spacemouse_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
