#include "compliant_controllers/ros2_controller.hpp"
#include <algorithm>
#include <cmath>

namespace compliant_controllers {
bool JointTaskSpaceController::validateAction(const Follow::Goal& goal, Execution& e, trajectory_msgs::msg::JointTrajectory& msg) {
  msg=goal.trajectory;
  std::string reason;
  if (!normalizeTrajectory(msg,joints_,arm_.continuous,arm_.lower,arm_.upper,reason) ||
      !goal.multi_dof_trajectory.points.empty() || !goal.component_path_tolerance.empty() ||
      !goal.component_goal_tolerance.empty()) return false;
  for (double v : msg.points.back().velocities) if (std::abs(v)>1e-6) return false;
  auto start=rclcpp::Time(msg.header.stamp,get_node()->get_clock()->get_clock_type());
  if (start.nanoseconds()!=0 && start+rclcpp::Duration(msg.points.back().time_from_start)<get_node()->now()) return false;
  e.path_p.fill(default_path_); e.goal_p.fill(default_goal_); e.goal_v.fill(default_velocity_);
  const auto tolerances=[&](const auto& list, auto& p, auto& v, auto& a) {
    std::vector<std::string> seen;
    for (const auto& t : list) {
      auto it=std::find(joints_.begin(),joints_.end(),t.name);
      if (it==joints_.end() || std::find(seen.begin(),seen.end(),t.name)!=seen.end()) return false;
      seen.push_back(t.name); auto i=std::distance(joints_.begin(),it);
      const auto assign=[](double x,double& out) {
        if (!std::isfinite(x) || (x<0 && x!=-1)) return false;
        if (x!=0) out=x==-1 ? 0 : x;
        return true;
      };
      if (!assign(t.position,p[i]) || !assign(t.velocity,v[i]) || !assign(t.acceleration,a[i])) return false;
    }
    return true;
  };
  // No measured acceleration interface: reject positive acceleration tolerances explicitly.
  for (const auto* list : {&goal.path_tolerance,&goal.goal_tolerance})
    for (const auto& t : *list) if (t.acceleration>0) return false;
  if (!tolerances(goal.path_tolerance,e.path_p,e.path_v,e.path_a) ||
      !tolerances(goal.goal_tolerance,e.goal_p,e.goal_v,e.goal_a)) return false;
  if (goal.goal_time_tolerance.sec<0 || goal.goal_time_tolerance.nanosec>=1000000000u) return false;
  e.grace=rclcpp::Duration(goal.goal_time_tolerance).seconds();
  if (e.grace==0) e.grace=default_grace_;
  return true;
}

void JointTaskSpaceController::configureAction() {
  auto n=get_node();
  default_path_=n->get_parameter("action_path_tolerance").as_double();
  default_goal_=n->get_parameter("action_goal_tolerance").as_double();
  default_velocity_=n->get_parameter("action_stopped_velocity_tolerance").as_double();
  default_grace_=n->get_parameter("action_goal_time_tolerance").as_double();
  for (double x : {default_path_,default_goal_,default_velocity_,default_grace_})
    if (!std::isfinite(x) || x<0) throw std::invalid_argument("invalid action tolerance default");
  action_server_=rclcpp_action::create_server<Follow>(n,"~/follow_joint_trajectory",
    [this](const auto& uuid, const auto goal) {
      std::lock_guard<std::mutex> lock(action_mutex_);
      const auto generation=generation_.load();
      Execution e; trajectory_msgs::msg::JointTrajectory msg;
      if (!accepting_.load() || !validateAction(*goal,e,msg))
        return rclcpp_action::GoalResponse::REJECT;
      accepted_generations_[uuid]=generation;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<GoalHandle> handle) {
      std::lock_guard<std::mutex> lock(action_mutex_);
      for (auto& e : executions_) if (e->handle==handle) {
        int expected=999;
        if (e->terminal.compare_exchange_strong(expected,-999))
          return rclcpp_action::CancelResponse::ACCEPT;
      }
      return rclcpp_action::CancelResponse::REJECT;
    },
    [this](const std::shared_ptr<GoalHandle> handle) {
      std::lock_guard<std::mutex> lock(action_mutex_);
      auto e=std::make_shared<Execution>(); e->handle=handle;
      e->generation=accepted_generations_.at(handle->get_goal_id());
      accepted_generations_.erase(handle->get_goal_id());
      auto msg=std::make_shared<TrajectoryMsg>();
      if (!accepting_.load() || e->generation!=generation_.load() || !validateAction(*handle->get_goal(),*e,*msg)) {
        auto result=std::make_shared<Follow::Result>(); result->error_code=Follow::Result::INVALID_GOAL;
        handle->abort(result); return;
      }
      for (auto& previous : executions_) { int expected=999; previous->terminal.compare_exchange_strong(expected, Follow::Result::INVALID_GOAL); }
      e->feedback.joint_names=joints_;
      for (auto* point : {&e->feedback.desired,&e->feedback.actual,&e->feedback.error}) {
        point->positions.resize(7); point->velocities.resize(7);
      }
      e->feedback.desired.accelerations.resize(7);
      executions_.push_back(e);
      pending_.writeFromNonRT(Pending{generation_.load(),msg,e});
    });
  action_timer_=n->create_wall_timer(std::chrono::milliseconds(20),[this]() {
    std::lock_guard<std::mutex> lock(action_mutex_);
    for (auto it=executions_.begin();it!=executions_.end();) {
      auto e=*it; int terminal=e->terminal.load();
      if (terminal==999 && (!accepting_.load() || e->generation!=generation_.load())) {
        int expected=999; e->terminal.compare_exchange_strong(expected,Follow::Result::INVALID_GOAL);
        terminal=e->terminal.load();
      }
      if (terminal==999) {
        std::lock_guard<std::mutex> feedback_lock(e->feedback_mutex);
        if (e->handle->is_executing()) e->handle->publish_feedback(std::make_shared<Follow::Feedback>(e->feedback));
        ++it; continue;
      }
      if (terminal==-999 && !e->handle->is_canceling()) {++it;continue;}
      auto result=std::make_shared<Follow::Result>();
      result->error_code=terminal==-999 ? Follow::Result::SUCCESSFUL : terminal;
      if (terminal==Follow::Result::INVALID_GOAL) result->error_string="Preempted, deactivated or controller update failed";
      if (terminal==Follow::Result::PATH_TOLERANCE_VIOLATED) result->error_string="Path tolerance violated; holding measured pose";
      if (terminal==Follow::Result::GOAL_TOLERANCE_VIOLATED) result->error_string="Goal tolerance deadline expired; holding measured pose";
      if (terminal==-999) e->handle->canceled(result);
      else if (terminal==Follow::Result::SUCCESSFUL) e->handle->succeed(result);
      else e->handle->abort(result);
      it=executions_.erase(it);
    }
  });
}

void JointTaskSpaceController::holdMeasuredReference() {
  trajectory_running_=false;
  for (std::size_t i=0;i<7;++i) {
    reference_.positions[i]=measured_.positions[arm_.indices[i]];
    reference_.velocities[i]=reference_.accelerations[i]=0;
  }
}

void JointTaskSpaceController::monitorAction(const rclcpp::Time& time) {
  auto e=active_execution_;
  if (!e) return;
  if (e->terminal.load()!=999) { holdMeasuredReference(); active_execution_.reset(); return; }
  int outcome=999;
  bool path_ok=true,goal_ok=true;
  const auto start=rclcpp::Time(active_msg_->header.stamp,time.get_clock_type());
  for (std::size_t i=0;i<7;++i) {
    auto j=arm_.indices[i];
    double p=reference_.positions[i]-measured_.positions[j];
    if (arm_.continuous[i]) p=std::remainder(p,2*M_PI);
    double v=reference_.velocities[i]-measured_.velocities[j];
    path_ok &= (e->path_p[i]<=0 || std::abs(p)<=e->path_p[i]) && (e->path_v[i]<=0 || std::abs(v)<=e->path_v[i]);
    goal_ok &= (e->goal_p[i]<=0 || std::abs(p)<=e->goal_p[i]) && (e->goal_v[i]<=0 || std::abs(v)<=e->goal_v[i]);
  }
  if (outcome==999 && time>=e->end) {
    if (goal_ok) outcome=Follow::Result::SUCCESSFUL;
    else if ((time-e->end).seconds()>e->grace) outcome=Follow::Result::GOAL_TOLERANCE_VIOLATED;
  } else if (outcome==999 && (start.nanoseconds()==0 || time>=start) && !path_ok) outcome=Follow::Result::PATH_TOLERANCE_VIOLATED;
  if (outcome!=999) {
    int expected=999; e->terminal.compare_exchange_strong(expected,outcome);
    if (e->terminal.load()!=Follow::Result::SUCCESSFUL) holdMeasuredReference();
    active_execution_.reset(); return;
  }
  if (cycles_%20==0) {
    std::unique_lock<std::mutex> lock(e->feedback_mutex,std::try_to_lock);
    if (lock.owns_lock()) {
      e->feedback.header.stamp=time;
      e->feedback.desired=reference_;
      for (std::size_t i=0;i<7;++i) {
        auto j=arm_.indices[i];
        e->feedback.actual.positions[i]=measured_.positions[j];
        e->feedback.actual.velocities[i]=measured_.velocities[j];
        e->feedback.error.positions[i]=reference_.positions[i]-measured_.positions[j];
        if (arm_.continuous[i]) e->feedback.error.positions[i]=std::remainder(e->feedback.error.positions[i],2*M_PI);
        e->feedback.error.velocities[i]=reference_.velocities[i]-measured_.velocities[j];
      }
    }
  }
}
}
