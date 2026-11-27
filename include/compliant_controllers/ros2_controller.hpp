#pragma once
#include "compliant_controllers/joint_task_space/controller.h"
#include "compliant_controllers/model.hpp"
#include <array>
#include <mutex>
#include <map>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
namespace compliant_controllers {
bool normalizeTrajectory(trajectory_msgs::msg::JointTrajectory& msg,
  const std::vector<std::string>& joints, const std::vector<bool>& continuous,
  const std::vector<double>& lower, const std::vector<double>& upper, std::string& reason);
class JointTaskSpaceController : public controller_interface::ControllerInterface {
public:
  ~JointTaskSpaceController() override { action_timer_.reset(); action_server_.reset(); }
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
private:
  using Follow = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;
  struct Execution {
    std::shared_ptr<GoalHandle> handle;
    std::uint64_t generation{0};
    std::atomic<int> terminal{999}; // result code; -999 means cancellation
    std::array<double,7> path_p{}, path_v{}, path_a{}, goal_p{}, goal_v{}, goal_a{};
    double grace{2.0};
    rclcpp::Time end{0,0,RCL_ROS_TIME};
    std::mutex feedback_mutex;
    Follow::Feedback feedback;
  };
  void configureAction();
  bool validateAction(const Follow::Goal&, Execution&, trajectory_msgs::msg::JointTrajectory&);
  void monitorAction(const rclcpp::Time&);
  void holdMeasuredReference();
  rclcpp_action::Server<Follow>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr action_timer_;
  std::mutex action_mutex_;
  std::vector<std::shared_ptr<Execution>> executions_;
  std::map<rclcpp_action::GoalUUID, std::uint64_t> accepted_generations_;
  std::shared_ptr<Execution> active_execution_;
  double default_path_{.5}, default_goal_{.03}, default_velocity_{.05}, default_grace_{2.0};
  struct Gains { std::array<Eigen::VectorXd, 10> values; };
  static const std::array<std::string, 10> gain_names_;
  std::shared_ptr<Gains> readGains();
  void applyGains(const Gains&);
  bool readState();
  bool softStopHolding() const;
  std::string soft_stop_state_prefix_;
  std::size_t soft_stop_state_index_{0};
  bool motion_paused_{false};
  std::vector<std::string> joints_;
  ArmModel arm_;
  std::unique_ptr<joint_task_space::CompliantController> core_;
  RobotState measured_{7}, desired_{7};
  std::array<std::array<std::size_t, 3>, 7> state_indices_{};
  std::array<std::size_t, 7> command_indices_{};
  Eigen::VectorXd feedback_filter_{Eigen::VectorXd::Zero(7)}, output_filter_{Eigen::VectorXd::Zero(7)};
  bool output_started_{false};
  bool trajectory_running_{false};
  double feedback_alpha_{1}, output_alpha_{1};
  int feedback_passes_{2};
  std::atomic<bool> accepting_{false};
  std::shared_ptr<Gains> gains_non_rt_, applied_gains_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Gains>> gains_buffer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  using TrajectoryMsg = trajectory_msgs::msg::JointTrajectory;
  struct Pending { std::uint64_t generation{0}; std::shared_ptr<TrajectoryMsg> message; std::shared_ptr<Execution> execution; };
  realtime_tools::RealtimeBuffer<Pending> pending_;
  std::atomic<std::uint64_t> generation_{0};
  std::shared_ptr<TrajectoryMsg> active_msg_;
  std::unique_ptr<joint_trajectory_controller::Trajectory> trajectory_;
  trajectory_msgs::msg::JointTrajectoryPoint reference_, sampled_;
  rclcpp::Subscription<TrajectoryMsg>::SharedPtr subscriber_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> effort_pub_;
  std::uint64_t cycles_{0};
};
}
