#pragma once
#include "compliant_controllers/model.hpp"
#include <array>
#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pinocchio/multibody/data.hpp>

namespace compliant_controllers {
// A diagnostic controller: no command interfaces, action servers or hardware mode changes.
class GravityMonitor : public controller_interface::ControllerInterface {
public:
  ~GravityMonitor() override { health_timer_.reset(); }
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
private:
  bool readState();
  std::vector<std::string> joints_;
  ArmModel arm_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::VectorXd q_, v_, effort_, q_pin_;
  std::array<std::array<std::size_t,3>,7> state_indices_{};
  using Comparison = control_msgs::msg::JointTrajectoryControllerState;
  std::unique_ptr<realtime_tools::RealtimePublisher<Comparison>> comparison_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_, quiet_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::atomic<bool> valid_{false}, quiet_{false}, active_{false};
  double velocity_limit_{.02}, quiet_duration_{.5}, quiet_elapsed_{0};
};
}
