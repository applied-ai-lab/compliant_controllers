#include "compliant_controllers/gravity_monitor.hpp"
#include "compliant_controllers/utils.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>

namespace compliant_controllers {
using CallbackReturn = controller_interface::CallbackReturn;
CallbackReturn GravityMonitor::on_init() {
  auto n=get_node();
  n->declare_parameter("joints",std::vector<std::string>{});
  n->declare_parameter("end_effector_link",std::string{});
  n->declare_parameter("quiet_velocity",.02);
  n->declare_parameter("quiet_duration",.5);
  return CallbackReturn::SUCCESS;
}
CallbackReturn GravityMonitor::on_configure(const rclcpp_lifecycle::State&) {
  try {
    auto n=get_node();
    joints_=n->get_parameter("joints").as_string_array();
    arm_=reduceToArm(get_robot_description(),joints_,n->get_parameter("end_effector_link").as_string());
    data_=std::make_unique<pinocchio::Data>(*arm_.model);
    q_.setZero(7);v_.setZero(7);effort_.setZero(7);q_pin_.setZero(arm_.model->nq);
    velocity_limit_=n->get_parameter("quiet_velocity").as_double();
    quiet_duration_=n->get_parameter("quiet_duration").as_double();
    if (!std::isfinite(velocity_limit_) || velocity_limit_<=0 ||
        !std::isfinite(quiet_duration_) || quiet_duration_<0) throw std::invalid_argument("invalid quasi-static thresholds");
    comparison_=std::make_unique<realtime_tools::RealtimePublisher<Comparison>>(
      n->create_publisher<Comparison>("~/comparison",10));
    auto& msg=comparison_->msg_;
    msg.joint_names=joints_;
    for (auto* point : {&msg.reference,&msg.feedback,&msg.error}) {
      point->positions.resize(7);point->velocities.resize(7);point->effort.resize(7);
    }
    // output remains empty: this monitor never generates a hardware command.
    valid_pub_=n->create_publisher<std_msgs::msg::Bool>("~/valid",10);
    quiet_pub_=n->create_publisher<std_msgs::msg::Bool>("~/quasi_static",10);
    health_timer_=n->create_wall_timer(std::chrono::milliseconds(100),[this]() {
      std_msgs::msg::Bool b;b.data=active_.load() && valid_.load();valid_pub_->publish(b);
      b.data=active_.load() && quiet_.load();quiet_pub_->publish(b);
    });
    return CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),"Gravity monitor configuration failed: %s",e.what());
    return CallbackReturn::ERROR;
  }
}
controller_interface::InterfaceConfiguration GravityMonitor::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE,{}};
}
controller_interface::InterfaceConfiguration GravityMonitor::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
  for (const auto& name:joints_) for (const auto* field:{"position","velocity","effort"}) c.names.push_back(name+"/"+field);
  return c;
}
bool GravityMonitor::readState() {
  for (std::size_t i=0;i<7;++i) {
    const auto j=arm_.indices[i];
    q_[j]=state_interfaces_[state_indices_[i][0]].get_optional().value_or(NAN);
    v_[j]=state_interfaces_[state_indices_[i][1]].get_optional().value_or(NAN);
    effort_[j]=state_interfaces_[state_indices_[i][2]].get_optional().value_or(NAN);
  }
  return q_.allFinite() && v_.allFinite() && effort_.allFinite();
}
CallbackReturn GravityMonitor::on_activate(const rclcpp_lifecycle::State&) {
  for (std::size_t i=0;i<7;++i) for (std::size_t k=0;k<3;++k) {
    auto name=joints_[i]+"/"+std::array<std::string,3>{"position","velocity","effort"}[k];
    auto it=std::find_if(state_interfaces_.begin(),state_interfaces_.end(),[&](auto& s){return s.get_name()==name;});
    if (it==state_interfaces_.end()) return CallbackReturn::ERROR;
    state_indices_[i][k]=std::distance(state_interfaces_.begin(),it);
  }
  if (!readState()) return CallbackReturn::ERROR;
  valid_=false;quiet_=false;quiet_elapsed_=0;active_=true;
  return CallbackReturn::SUCCESS;
}
CallbackReturn GravityMonitor::on_deactivate(const rclcpp_lifecycle::State&) {
  active_=false;valid_=false;quiet_=false;
  return CallbackReturn::SUCCESS;
}
controller_interface::return_type GravityMonitor::update(const rclcpp::Time& time,const rclcpp::Duration& period) {
  try {
    if (!readState() || !std::isfinite(period.seconds()) || period.seconds()<=0)
      throw std::runtime_error("invalid monitor input");
    const auto& gravity=computeGravity(q_,*arm_.model,*data_,q_pin_);
    if (!gravity.allFinite()) throw std::runtime_error("non-finite model gravity");
    valid_=true;
    quiet_elapsed_=v_.cwiseAbs().maxCoeff()<=velocity_limit_ ? quiet_elapsed_+period.seconds() : 0;
    quiet_=quiet_elapsed_>=quiet_duration_;
    if (comparison_->trylock()) {
      auto& msg=comparison_->msg_;msg.header.stamp=time;
      for (std::size_t i=0;i<7;++i) {
        auto j=arm_.indices[i];
        msg.reference.positions[i]=msg.feedback.positions[i]=q_[j];
        msg.reference.velocities[i]=0;msg.feedback.velocities[i]=v_[j];
        msg.reference.effort[i]=gravity[j];msg.feedback.effort[i]=effort_[j];
        msg.error.positions[i]=0;msg.error.velocities[i]=-v_[j];
        msg.error.effort[i]=gravity[j]-effort_[j];
      }
      comparison_->unlockAndPublish();
    }
  } catch (const std::exception& e) {
    valid_=false;quiet_=false;quiet_elapsed_=0;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),2000,"Gravity monitor skipped sample: %s",e.what());
  }
  // A diagnostic failure must not request changes to the motion controller.
  return controller_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(compliant_controllers::GravityMonitor,controller_interface::ControllerInterface)
