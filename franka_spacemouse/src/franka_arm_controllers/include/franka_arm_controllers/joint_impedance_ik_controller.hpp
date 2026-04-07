// Copyright (c) 2025 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <franka_arm_controllers/robot_utils.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_arm_controllers {

/**
 * joint impedance controller to move the robot to a desired pose.
 */
class JointImpedanceIKController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  /**
   * @brief updates the joint states from the state interfaces
   *
   */
  bool update_joint_states_();
  bool resolve_joint_state_interface_indices_();
  void set_zero_torque_commands_();

  /**
   * @brief computes the torque commands based on impedance control law with compensated coriolis
   * terms
   *
   * @return Eigen::Vector7d torque for each joint of the robot
   */
  Vector7d compute_torque_command_(const Vector7d& joint_positions_desired,
                                   const Vector7d& joint_positions_current,
                                   const Vector7d& joint_velocities_current);

  /**
   * @brief assigns the Kp, Kd and arm_id parameters
   *
   * @return true when parameters are present, false when parameters are not available
   */
  bool assign_parameters_();
  bool update_current_pose_from_joint_states_();

  tf2::Vector3 transform_velocity_to_world_frame_(
      const geometry_msgs::msg::Twist& msg) const;

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;

  const bool k_elbow_activated_{false};

  std::string arm_id_;
  std::string namespace_prefix_;
  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  unsigned int nj_;
  KDL::JntArray q_min_, q_max_, q_init_, q_result_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
  int ik_max_iterations_{100};
  double ik_eps_{1e-6};
  void solve_ik_(const Eigen::Vector3d& new_position, const Eigen::Quaterniond& new_orientation);
  bool is_gripper_loaded_ = true;
  bool use_joint_state_fk_fallback_{false};
  std::vector<double> arm_mounting_orientation_;

  std::string robot_description_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  const int num_joints_{7};

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_efforts_current_{0, 0, 0, 0, 0, 0, 0};

  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  Eigen::Quaterniond target_orientation_{1.0, 0.0, 0.0, 0.0};
  bool target_pose_initialized_{false};
  bool first_update_after_activate_{true};
  std::atomic<bool> spacemouse_message_received_{false};
  std::atomic<int64_t> last_spacemouse_msg_steady_time_ns_{0};
  int64_t activation_steady_time_ns_{0};
  double startup_hold_sec_{0.40};
  double command_timeout_sec_{0.2};
  double max_dt_for_integration_sec_{0.01};
  double max_linear_speed_{0.006};
  double max_angular_speed_{0.03};
  double input_deadband_{0.10};
  double input_smoothing_alpha_{0.2};
  bool linear_dominant_angular_suppression_enabled_{true};
  double linear_dominant_angular_suppression_ratio_{1.8};
  double linear_dominant_angular_suppression_min_linear_norm_{0.02};
  double linear_dominant_angular_suppression_scale_{0.0};
  bool linear_dominant_orientation_lock_enabled_{true};
  std::string teleop_translation_frame_{"base"};
  double ik_seed_current_state_blend_{0.8};
  double ik_posture_bias_{0.0};
  bool keep_active_on_transient_update_failure_{true};
  int safe_hold_velocity_violation_count_threshold_{6};
  double zero_input_velocity_epsilon_{1e-6};
  Eigen::Vector3d max_cartesian_error_{0.010, 0.010, 0.010};
  double max_orientation_error_rad_{0.10};
  std::array<double, 7> max_joint_velocity_limits_{{0.20, 0.20, 0.20, 0.20, 0.25, 0.25, 0.30}};
  std::array<double, 7> max_measured_joint_velocity_for_safe_hold_{
      {0.6, 0.6, 0.6, 0.6, 0.7, 0.7, 0.8}};
  std::array<double, 7> max_joint_position_error_for_torque_{
      {0.05, 0.05, 0.05, 0.05, 0.07, 0.07, 0.07}};
  std::array<double, 7> max_torque_command_limits_{{30.0, 30.0, 30.0, 30.0, 8.0, 8.0, 8.0}};
  std::array<double, 7> max_torque_rate_limits_{{200.0, 200.0, 200.0, 200.0, 120.0, 120.0, 120.0}};
  std::array<double, 7> previous_torque_command_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> preferred_joint_posture_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<int, 7> joint_position_state_interface_indices_{{-1, -1, -1, -1, -1, -1, -1}};
  std::array<int, 7> joint_velocity_state_interface_indices_{{-1, -1, -1, -1, -1, -1, -1}};
  std::array<int, 7> joint_effort_state_interface_indices_{{-1, -1, -1, -1, -1, -1, -1}};
  bool interfaces_resolved_{false};
  bool state_read_valid_{false};
  int measured_velocity_violation_count_{0};

  // Spacemouse
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr spacemouse_sub_;
  void spacemouse_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  std::atomic<uint64_t> command_state_sequence_{0};
  std::array<std::atomic<double>, 3> desired_linear_velocity_cmd_{
      {0.0, 0.0, 0.0}};
  std::array<std::atomic<double>, 3> desired_angular_velocity_cmd_{
      {0.0, 0.0, 0.0}};
};
}  // namespace franka_arm_controllers
