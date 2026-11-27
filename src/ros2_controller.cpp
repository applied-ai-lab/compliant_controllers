#include "compliant_controllers/ros2_controller.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>

namespace compliant_controllers {
using CallbackReturn = controller_interface::CallbackReturn;
const std::array<std::string, 10> JointTaskSpaceController::gain_names_{
  "joint_stiffness", "rotor_inertia", "joint_k", "joint_d", "task_k", "task_d",
  "friction_l", "friction_lp", "friction_li", "max_joint_error"};

bool normalizeTrajectory(trajectory_msgs::msg::JointTrajectory& msg,
  const std::vector<std::string>& joints, const std::vector<bool>& continuous,
  const std::vector<double>& lower, const std::vector<double>& upper, std::string& reason) {
  const auto fail = [&](const char* why) { reason = why; return false; };
  const auto n = joints.size();
  if (msg.joint_names.size() != n || std::set<std::string>(msg.joint_names.begin(), msg.joint_names.end()).size() != n)
    return fail("all seven unique joint names required");
  std::vector<std::size_t> order;
  for (const auto& name : joints) {
    auto it = std::find(msg.joint_names.begin(), msg.joint_names.end(), name);
    if (it == msg.joint_names.end()) return fail("joint names do not match this arm");
    order.push_back(std::distance(msg.joint_names.begin(), it));
  }
  if (msg.points.empty()) return fail("empty trajectory; deactivate controller to stop");
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u) return fail("invalid header timestamp");
  std::int64_t previous = -1;
  for (auto& point : msg.points) {
    if (point.positions.size() != n || !point.effort.empty()) return fail("positions required; no feedforward effort input");
    if (!point.accelerations.empty() && point.velocities.empty()) return fail("acceleration requires velocity");
    for (auto* field : {&point.positions, &point.velocities, &point.accelerations}) {
      if (field->empty()) continue;
      if (field->size() != n || !std::all_of(field->begin(), field->end(), [](double x){return std::isfinite(x);}))
        return fail("trajectory arrays must have seven finite values");
      const auto copy = *field;
      for (std::size_t i = 0; i < n; ++i) (*field)[i] = copy[order[i]];
    }
    for (std::size_t i = 0; i < n; ++i)
      if (!continuous[i] && (point.positions[i] < lower[i] || point.positions[i] > upper[i]))
        return fail("bounded joint target outside URDF limits");
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u) return fail("invalid point time");
    const auto t = std::int64_t(point.time_from_start.sec) * 1000000000 + point.time_from_start.nanosec;
    if (t <= previous) return fail("point times must increase strictly");
    previous = t;
  }
  msg.joint_names = joints;
  return true;
}

CallbackReturn JointTaskSpaceController::on_init() {
  auto n = get_node();
  n->declare_parameter("soft_stop_state_prefix", std::string{});
  n->declare_parameter("action_path_tolerance", .5);
  n->declare_parameter("action_goal_tolerance", .03);
  n->declare_parameter("action_stopped_velocity_tolerance", .05);
  n->declare_parameter("action_goal_time_tolerance", 2.0);
  n->declare_parameter("joints", std::vector<std::string>{});
  n->declare_parameter("end_effector_link", std::string{});
  n->declare_parameter("feedback_cutoff_hz", 30.0);
  n->declare_parameter("feedback_filter_passes", 2); // ROS1 performed two updates per read
  n->declare_parameter("output_cutoff_hz", 100.0);
  const std::array<double, 10> defaults{3750, .3, 1, 2, 10, .1, 60, 4, 0, 0};
  for (std::size_t i = 0; i < gain_names_.size(); ++i)
    n->declare_parameter(gain_names_[i], std::vector<double>((i == 4 || i == 5) ? 6 : 7, defaults[i]));
  return CallbackReturn::SUCCESS;
}

std::shared_ptr<JointTaskSpaceController::Gains> JointTaskSpaceController::readGains() {
  auto result = std::make_shared<Gains>();
  for (std::size_t i = 0; i < gain_names_.size(); ++i) {
    auto v = get_node()->get_parameter(gain_names_[i]).as_double_array();
    if (v.size() != ((i == 4 || i == 5) ? 6u : 7u)) throw std::invalid_argument("wrong gain array size: " + gain_names_[i]);
    result->values[i] = Eigen::Map<Eigen::VectorXd>(v.data(), v.size());
    if (!result->values[i].allFinite() || (result->values[i].array() < 0).any() ||
        (i < 2 && (result->values[i].array() <= 0).any())) throw std::invalid_argument("invalid gains: " + gain_names_[i]);
  }
  return result;
}

void JointTaskSpaceController::applyGains(const Gains& g) {
  const auto& v = g.values;
  const auto ordered = [&](std::size_t k) {
    Eigen::VectorXd values(7);
    for (std::size_t i = 0; i < 7; ++i) values[arm_.indices[i]] = v[k][i];
    return values;
  };
  if (!core_->setJointStiffnessMatrix(ordered(0).asDiagonal()) || !core_->setRotorInertiaMatrix(ordered(1).asDiagonal()) ||
      !core_->setJointKMatrix(ordered(2).asDiagonal()) || !core_->setJointDMatrix(ordered(3).asDiagonal()) ||
      !core_->setTaskKMatrix(v[4].asDiagonal()) || !core_->setTaskDMatrix(v[5].asDiagonal()) ||
      !core_->setFrictionL(ordered(6).asDiagonal()) || !core_->setFrictionLp(ordered(7).asDiagonal()) ||
      !core_->setFrictionLi(ordered(8).asDiagonal()) || !core_->setMaxJointError(ordered(9)))
    throw std::invalid_argument("invalid control matrices");
}

CallbackReturn JointTaskSpaceController::on_configure(const rclcpp_lifecycle::State&) {
  try {
    parameter_callback_.reset();
    soft_stop_state_prefix_ = get_node()->get_parameter("soft_stop_state_prefix").as_string();
    if (get_update_rate() != 1000) throw std::invalid_argument("legacy observer requires update_rate: 1000");
    joints_ = get_node()->get_parameter("joints").as_string_array();
    const auto tool = get_node()->get_parameter("end_effector_link").as_string();
    arm_ = reduceToArm(get_robot_description(), joints_, tool);
    core_ = std::make_unique<joint_task_space::CompliantController>(std::move(arm_.model), tool, 7, true);
    gains_non_rt_ = readGains(); applyGains(*gains_non_rt_);
    gains_buffer_.writeFromNonRT(gains_non_rt_); applied_gains_ = gains_non_rt_;
    auto alpha = [&](const char* name) {
      const double f = get_node()->get_parameter(name).as_double();
      if (!std::isfinite(f) || f < 0 || f > 500) throw std::invalid_argument("cutoff must be in [0,500] Hz");
      return f == 0 ? 1.0 : 2 * M_PI * f / (1000 + 2 * M_PI * f);
    };
    feedback_alpha_ = alpha("feedback_cutoff_hz"); output_alpha_ = alpha("output_cutoff_hz");
    feedback_passes_ = get_node()->get_parameter("feedback_filter_passes").as_int();
    if (feedback_passes_ < 1 || feedback_passes_ > 2) throw std::invalid_argument("feedback_filter_passes must be 1 or 2");
    trajectory_ = std::make_unique<joint_trajectory_controller::Trajectory>();
    for (auto* p : {&reference_, &sampled_}) {
      p->positions.resize(7); p->velocities.resize(7); p->accelerations.resize(7);
    }
    configureAction();
    subscriber_ = get_node()->create_subscription<TrajectoryMsg>("~/joint_trajectory", rclcpp::QoS(1),
      [this](TrajectoryMsg::SharedPtr msg) {
        const auto generation = generation_.load();
        if (!accepting_.load()) return;
        std::lock_guard<std::mutex> lock(action_mutex_);
        for (const auto& e : executions_) if (e->terminal.load() == 999) {
          RCLCPP_WARN(get_node()->get_logger(), "Trajectory topic ignored while an action owns the reference");
          return;
        }
        std::string reason;
        if (!normalizeTrajectory(*msg, joints_, arm_.continuous, arm_.lower, arm_.upper, reason)) {
          RCLCPP_WARN(get_node()->get_logger(), "Rejected trajectory: %s", reason.c_str()); return;
        }
        const auto start = rclcpp::Time(msg->header.stamp, get_node()->get_clock()->get_clock_type());
        if (start.nanoseconds() != 0 && start + rclcpp::Duration(msg->points.back().time_from_start) < get_node()->now()) {
          RCLCPP_WARN(get_node()->get_logger(), "Rejected expired trajectory"); return;
        }
        pending_.writeFromNonRT(Pending{generation, msg});
      });
    effort_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/commanded_effort", 10));
    effort_pub_->msg_.data.resize(7);
    parameter_callback_ = get_node()->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
      rcl_interfaces::msg::SetParametersResult result; result.successful = false;
      try {
        auto next = std::make_shared<Gains>(*gains_non_rt_);
        for (const auto& p : params) {
          auto it = std::find(gain_names_.begin(), gain_names_.end(), p.get_name());
          if (it == gain_names_.end()) throw std::invalid_argument("only gain arrays can change at runtime");
          const auto k = std::distance(gain_names_.begin(), it);
          const auto values = p.as_double_array();
          if (values.size() != static_cast<std::size_t>(next->values[k].size())) throw std::invalid_argument("gain array size");
          next->values[k] = Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
          if (!next->values[k].allFinite() || (next->values[k].array() < 0).any() ||
              (k < 2 && (next->values[k].array() <= 0).any())) throw std::invalid_argument("invalid gains");
        }
        gains_non_rt_ = next; gains_buffer_.writeFromNonRT(next); result.successful = true;
      } catch (const std::exception& e) { result.reason = e.what(); }
      return result;
    });
    return CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configure failed: %s", e.what()); return CallbackReturn::ERROR;
  }
}

controller_interface::InterfaceConfiguration JointTaskSpaceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto& j : joints_) c.names.push_back(j + "/effort");
  return c;
}
controller_interface::InterfaceConfiguration JointTaskSpaceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto& j : joints_) for (const auto* s : {"position", "velocity", "effort"}) c.names.push_back(j + "/" + s);
  if (!soft_stop_state_prefix_.empty()) c.names.push_back(soft_stop_state_prefix_ + "/holding");
  return c;
}
bool JointTaskSpaceController::softStopHolding() const {
  if (soft_stop_state_prefix_.empty()) return false;
  const double state = state_interfaces_[soft_stop_state_index_].get_optional().value_or(NAN);
  return !std::isfinite(state) || state != 0.0;
}

bool JointTaskSpaceController::readState() {
  for (std::size_t i = 0; i < 7; ++i) {
    const auto j = arm_.indices[i];
    measured_.positions[j] = state_interfaces_[state_indices_[i][0]].get_optional().value_or(NAN);
    measured_.velocities[j] = state_interfaces_[state_indices_[i][1]].get_optional().value_or(NAN);
    measured_.efforts[j] = state_interfaces_[state_indices_[i][2]].get_optional().value_or(NAN);
  }
  return measured_.positions.allFinite() && measured_.velocities.allFinite() && measured_.efforts.allFinite();
}
CallbackReturn JointTaskSpaceController::on_activate(const rclcpp_lifecycle::State&) {
  accepting_.store(false);
  ++generation_;
  try {
    for (std::size_t i = 0; i < 7; ++i) {
      for (std::size_t k = 0; k < 3; ++k) {
        const auto name = joints_[i] + "/" + std::array<std::string,3>{"position","velocity","effort"}[k];
        auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](auto& s){return s.get_name() == name;});
        if (it == state_interfaces_.end()) throw std::runtime_error("missing state: " + name);
        state_indices_[i][k] = std::distance(state_interfaces_.begin(), it);
      }
      auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&](auto& c){return c.get_name() == joints_[i] + "/effort";});
      if (it == command_interfaces_.end()) throw std::runtime_error("missing effort command");
      command_indices_[i] = std::distance(command_interfaces_.begin(), it);
    }
    if (!soft_stop_state_prefix_.empty()) {
      const auto name = soft_stop_state_prefix_ + "/holding";
      auto state = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto& interface) { return interface.get_name() == name; });
      if (state == state_interfaces_.end()) throw std::runtime_error("missing state: " + name);
      soft_stop_state_index_ = std::distance(state_interfaces_.begin(), state);
    }
    if (!readState()) throw std::runtime_error("non-finite activation feedback");
    (void)core_->init();
    desired_ = measured_; desired_.velocities.setZero(); desired_.efforts.setZero();
    feedback_filter_ = measured_.efforts; output_started_ = false;
    for (std::size_t i = 0; i < 7; ++i) reference_.positions[i] = measured_.positions[arm_.indices[i]];
    std::fill(reference_.velocities.begin(), reference_.velocities.end(), 0);
    std::fill(reference_.accelerations.begin(), reference_.accelerations.end(), 0);
    pending_.writeFromNonRT(Pending{}); active_msg_.reset(); active_execution_.reset(); trajectory_running_=false; cycles_ = 0;
    motion_paused_ = softStopHolding();
    accepting_.store(!motion_paused_); return CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Activate failed: %s", e.what()); return CallbackReturn::ERROR;
  }
}
CallbackReturn JointTaskSpaceController::on_deactivate(const rclcpp_lifecycle::State&) {
  accepting_.store(false);
  ++generation_;
  if (active_execution_) { int expected=999; active_execution_->terminal.compare_exchange_strong(expected, Follow::Result::INVALID_GOAL); }
  // Physical stop/hold belongs to the hardware's effort -> POSITION mode switch.
  return CallbackReturn::SUCCESS;
}
controller_interface::return_type JointTaskSpaceController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  try {
    if (!readState()) throw std::runtime_error("non-finite hardware feedback");
    if (softStopHolding() || motion_paused_) {
      if (accepting_.exchange(false)) ++generation_;
      motion_paused_ = true;
      holdMeasuredReference();
      active_msg_.reset();
      if (active_execution_) {
        int expected = 999;
        active_execution_->terminal.compare_exchange_strong(expected, Follow::Result::INVALID_GOAL);
        active_execution_.reset();
      }
      // The driver owns the powered POSITION hold. Reactivation resets the observer
      // and reference at the measured pose before torque commands are accepted again.
      return controller_interface::return_type::OK;
    }
    auto gains = *gains_buffer_.readFromRT();
    if (gains != applied_gains_) { applyGains(*gains); applied_gains_ = gains; }
    const auto pending = *pending_.readFromRT();
    auto msg = pending.generation == generation_.load() ? pending.message : nullptr;
    if (msg && msg != active_msg_) {
      trajectory_->update(msg);
      trajectory_->set_point_before_trajectory_msg(time, reference_, arm_.continuous);
      active_msg_ = msg; trajectory_running_=true;
      active_execution_ = pending.execution;
      if (active_execution_) {
        auto start = rclcpp::Time(msg->header.stamp, time.get_clock_type());
        if (start.nanoseconds()==0) start=time;
        active_execution_->end=start+rclcpp::Duration(msg->points.back().time_from_start);
      }
    }
    if (active_msg_ && trajectory_running_) {
      joint_trajectory_controller::TrajectoryPointConstIter begin, end;
      if (trajectory_->sample(time, joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE,
                              sampled_, begin, end)) {
        sampled_.velocities.resize(7, 0); sampled_.accelerations.resize(7, 0);
        reference_ = sampled_;
      }
    }
    monitorAction(time);
    for (std::size_t i = 0; i < 7; ++i) {
      desired_.positions[arm_.indices[i]] = reference_.positions[i];
      desired_.velocities[arm_.indices[i]] = reference_.velocities[i];
      desired_.accelerations[arm_.indices[i]] = reference_.accelerations[i];
    }
    for (int pass = 0; pass < feedback_passes_; ++pass) feedback_filter_ += feedback_alpha_ * (measured_.efforts - feedback_filter_);
    measured_.efforts = feedback_filter_;
    const auto& command = core_->computeEffort(desired_, measured_, period.seconds());
    if (!output_started_) { output_filter_ = command; output_started_ = true; }
    else output_filter_ += output_alpha_ * (command - output_filter_);
    if (!output_filter_.allFinite()) throw std::runtime_error("non-finite filtered effort");
    for (std::size_t i = 0; i < 7; ++i)
      if (!command_interfaces_[command_indices_[i]].set_value(output_filter_[arm_.indices[i]])) throw std::runtime_error("effort interface unavailable");
    if (++cycles_ % 10 == 0 && effort_pub_->trylock()) {
      for (std::size_t i = 0; i < 7; ++i) effort_pub_->msg_.data[i] = output_filter_[arm_.indices[i]];
      effort_pub_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  } catch (const std::exception& e) {
    accepting_.store(false);
    if (active_execution_) { int expected=999; active_execution_->terminal.compare_exchange_strong(expected, Follow::Result::INVALID_GOAL); }
    RCLCPP_ERROR(get_node()->get_logger(), "Update failed; requesting hardware hold: %s", e.what());
    return controller_interface::return_type::ERROR;
  }
}
}
PLUGINLIB_EXPORT_CLASS(compliant_controllers::JointTaskSpaceController, controller_interface::ControllerInterface)
