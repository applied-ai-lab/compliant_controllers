#include <gtest/gtest.h>
#include <fstream>
#include <limits>
#include <controller_interface/controller_interface_params.hpp>
#include "compliant_controllers/ros2_controller.hpp"
#include "compliant_controllers/gravity_monitor.hpp"
using namespace compliant_controllers;
using Msg=trajectory_msgs::msg::JointTrajectory;
std::vector<std::string> names() {
  std::vector<std::string> n; for(int i=1;i<=7;++i)n.push_back("right_kinova_arm_joint_"+std::to_string(i)); return n;
}
Msg message() {
  Msg m; m.joint_names=names(); m.points.resize(1); m.points[0].positions.assign(7,.1);
  m.points[0].time_from_start.nanosec=25000000; return m;
}
bool valid(Msg& m) {
  std::string why;
  return normalizeTrajectory(m,names(),{true,false,true,false,true,false,true},std::vector<double>(7,-2),std::vector<double>(7,2),why);
}
TEST(Reference, ReordersNamesAndAcceptsNativeGelloPositionOnlyPoint) {
  auto m=message(); std::reverse(m.joint_names.begin(),m.joint_names.end());
  m.points[0].positions={.1,.2,.3,.4,.5,.6,.7}; ASSERT_TRUE(valid(m));
  EXPECT_DOUBLE_EQ(m.points[0].positions[0],.7); EXPECT_EQ(m.joint_names,names());
}
TEST(Reference, RejectsPartialDuplicateNanDerivativesAndUnorderedTimes) {
  auto m=message();m.points[0].positions.pop_back();EXPECT_FALSE(valid(m));
  m=message();m.joint_names[0]=m.joint_names[1];EXPECT_FALSE(valid(m));
  m=message();m.points[0].positions[0]=NAN;EXPECT_FALSE(valid(m));
  m=message();m.points[0].velocities.assign(6,0);EXPECT_FALSE(valid(m));
  m=message();m.points.push_back(m.points[0]);EXPECT_FALSE(valid(m));
  m=message();m.points[0].positions[1]=3;EXPECT_FALSE(valid(m));
  m=message();m.points[0].accelerations.assign(7,0);EXPECT_FALSE(valid(m));
}
TEST(Reference, Samples25msLinearAnd100msQuinticHorizons) {
  for(bool quintic:{false,true}) {
    auto m=std::make_shared<Msg>(message());m->points[0].positions.assign(7,1);
    m->points[0].time_from_start.nanosec=quintic?100000000:25000000;
    if(quintic){m->points[0].velocities.assign(7,0);m->points[0].accelerations.assign(7,0);}
    trajectory_msgs::msg::JointTrajectoryPoint start,out;start.positions.assign(7,0);
    start.velocities.assign(7,0);start.accelerations.assign(7,0);
    joint_trajectory_controller::Trajectory t(m);
    const rclcpp::Time now(1000000000LL,RCL_ROS_TIME);
    t.set_point_before_trajectory_msg(now,start);
    joint_trajectory_controller::TrajectoryPointConstIter a,b;
    ASSERT_TRUE(t.sample(now,joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,out,a,b));
    ASSERT_TRUE(t.sample(now+rclcpp::Duration::from_seconds(quintic?.05:.0125),joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,out,a,b));
    EXPECT_NEAR(out.positions[0],.5,1e-12);
    EXPECT_NEAR(out.velocities[0],quintic?18.75:40,1e-10);
  }
}
class Lifecycle : public ::testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
};
TEST_F(Lifecycle, ClaimsEffortRejectsBadRateAndReseedsOnReactivation) {
  std::ifstream file(std::string(TEST_SOURCE_DIR)+"/urdf/oxf20_right_arm.urdf");
  const std::string xml{std::istreambuf_iterator<char>(file),{}};
  JointTaskSpaceController c;
  controller_interface::ControllerInterfaceParams p;
  p.controller_name="test_compliant";p.robot_description=xml;p.update_rate=1000;p.controller_manager_update_rate=1000;
  ASSERT_EQ(c.init(p),controller_interface::return_type::OK);
  c.get_node()->set_parameters({rclcpp::Parameter("joints",names()),rclcpp::Parameter("end_effector_link",std::string("right_kinova_arm_tool_frame"))});
  ASSERT_EQ(c.on_configure(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(c.command_interface_configuration().names.size(),7u);
  EXPECT_EQ(c.state_interface_configuration().names.size(),21u);
  std::array<std::array<double,3>,7> feedback{};std::array<double,7> output{};
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  std::vector<hardware_interface::LoanedStateInterface> ls;
  std::vector<hardware_interface::LoanedCommandInterface> lc;
  for(int i=0;i<7;++i){
    for(int k=0;k<3;++k){
      states.push_back(std::make_shared<hardware_interface::StateInterface>(names()[i],std::array<std::string,3>{"position","velocity","effort"}[k],&feedback[i][k]));
      ls.emplace_back(states.back());
    }
    commands.push_back(std::make_shared<hardware_interface::CommandInterface>(names()[i],"effort",&output[i]));lc.emplace_back(commands.back(),nullptr);
  }
  c.assign_interfaces(std::move(lc),std::move(ls));
  ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(c.update(rclcpp::Time(1000000000LL),rclcpp::Duration::from_seconds(.001)),controller_interface::return_type::OK);
  for(double x:output)EXPECT_TRUE(std::isfinite(x));
  feedback[0][0]=NAN;
  EXPECT_EQ(c.update(rclcpp::Time(1001000000LL),rclcpp::Duration::from_seconds(.001)),controller_interface::return_type::ERROR);
  c.on_deactivate(rclcpp_lifecycle::State{});feedback[0][0]=.7;
  ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(c.update(rclcpp::Time(2000000000LL),rclcpp::Duration::from_seconds(.001)),controller_interface::return_type::OK);
  auto result=c.get_node()->set_parameters_atomically({rclcpp::Parameter("rotor_inertia",std::vector<double>(7,0))});
  EXPECT_FALSE(result.successful);
  EXPECT_TRUE(c.get_node()->set_parameters_atomically({rclcpp::Parameter("task_k",std::vector<double>(6,20))}).successful);
  c.on_deactivate(rclcpp_lifecycle::State{});c.release_interfaces();
  JointTaskSpaceController bad;p.controller_name="wrong_rate";p.update_rate=500;
  ASSERT_EQ(bad.init(p),controller_interface::return_type::OK);
  bad.get_node()->set_parameters({rclcpp::Parameter("joints",names()),rclcpp::Parameter("end_effector_link",std::string("right_kinova_arm_tool_frame"))});
  EXPECT_EQ(bad.on_configure(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::ERROR);
}

TEST_F(Lifecycle, GravityMonitorNeedsNoCommandsAndInvalidFeedbackStaysDiagnostic) {
  std::ifstream file(std::string(TEST_SOURCE_DIR)+"/urdf/oxf20_right_arm.urdf");
  const std::string xml{std::istreambuf_iterator<char>(file),{}};
  GravityMonitor c;
  controller_interface::ControllerInterfaceParams p;
  p.controller_name="test_gravity_monitor";p.robot_description=xml;p.update_rate=100;p.controller_manager_update_rate=1000;
  ASSERT_EQ(c.init(p),controller_interface::return_type::OK);
  auto ordered=names();std::reverse(ordered.begin(),ordered.end());
  c.get_node()->set_parameters({rclcpp::Parameter("joints",ordered),rclcpp::Parameter("end_effector_link",std::string("right_kinova_arm_tool_frame"))});
  ASSERT_EQ(c.on_configure(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(c.command_interface_configuration().type,controller_interface::interface_configuration_type::NONE);
  EXPECT_TRUE(c.command_interface_configuration().names.empty());
  EXPECT_EQ(c.state_interface_configuration().names.size(),21u);
  std::array<std::array<double,3>,7> feedback{};
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
  std::vector<hardware_interface::LoanedStateInterface> loaned;
  for (int i=0;i<7;++i) for(int k=2;k>=0;--k) {
    states.push_back(std::make_shared<hardware_interface::StateInterface>(names()[i],std::array<std::string,3>{"position","velocity","effort"}[k],&feedback[i][k]));
    loaned.emplace_back(states.back());
  }
  c.assign_interfaces({},std::move(loaned));
  ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  const auto now=rclcpp::Time(1000000000LL);
  EXPECT_EQ(c.update(now,rclcpp::Duration::from_seconds(.01)),controller_interface::return_type::OK);
  feedback[3][2]=NAN;
  EXPECT_EQ(c.update(now,rclcpp::Duration::from_seconds(.01)),controller_interface::return_type::OK);
  c.on_deactivate(rclcpp_lifecycle::State{});
  EXPECT_EQ(c.on_activate(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::ERROR);
  feedback[3][2]=0;
  ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(c.update(now,rclcpp::Duration::from_seconds(0)),controller_interface::return_type::OK);
  EXPECT_EQ(c.update(now,rclcpp::Duration::from_seconds(.01)),controller_interface::return_type::OK);
  c.on_deactivate(rclcpp_lifecycle::State{});c.release_interfaces();
}

TEST_F(Lifecycle, PoweredHoldPausesUntilReactivationAndDiscardsOldReference) {
  std::ifstream file(std::string(TEST_SOURCE_DIR)+"/urdf/oxf20_right_arm.urdf");
  const std::string xml{std::istreambuf_iterator<char>(file),{}};
  JointTaskSpaceController c;
  controller_interface::ControllerInterfaceParams p;
  p.controller_name="powered_hold";p.robot_description=xml;p.update_rate=1000;p.controller_manager_update_rate=1000;
  ASSERT_EQ(c.init(p),controller_interface::return_type::OK);
  c.get_node()->set_parameters({rclcpp::Parameter("joints",names()),
    rclcpp::Parameter("end_effector_link",std::string("right_kinova_arm_tool_frame")),
    rclcpp::Parameter("soft_stop_state_prefix",std::string("soft_stop"))});
  ASSERT_EQ(c.on_configure(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(c.state_interface_configuration().names.size(),22u);
  double holding=0;
  std::array<std::array<double,3>,7> feedback{};
  std::array<double,7> output{};
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  std::vector<hardware_interface::LoanedStateInterface> ls;
  std::vector<hardware_interface::LoanedCommandInterface> lc;
  for(int i=0;i<7;++i) {
    for(int k=0;k<3;++k) {
      states.push_back(std::make_shared<hardware_interface::StateInterface>(names()[i],
        std::array<std::string,3>{"position","velocity","effort"}[k],&feedback[i][k]));
      ls.emplace_back(states.back());
    }
    commands.push_back(std::make_shared<hardware_interface::CommandInterface>(names()[i],"effort",&output[i]));
    lc.emplace_back(commands.back(),nullptr);
  }
  states.push_back(std::make_shared<hardware_interface::StateInterface>("soft_stop","holding",&holding));
  ls.emplace_back(states.back());
  c.assign_interfaces(std::move(lc),std::move(ls));
  ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  const auto period=rclcpp::Duration::from_seconds(.001);
  ASSERT_EQ(c.update(rclcpp::Time(1000000000LL),period),controller_interface::return_type::OK);
  const auto previous=output;
  using Follow = control_msgs::action::FollowJointTrajectory;
  auto client = rclcpp_action::create_client<Follow>(c.get_node(), "~/follow_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(c.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  Follow::Goal goal;goal.trajectory=message();
  auto send_goal = [&]() {
    auto future = client->async_send_goal(goal);
    EXPECT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(2)),
      rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  auto active_goal=send_goal();
  ASSERT_NE(active_goal,nullptr);
  auto result=client->async_get_result(active_goal);
  holding=1;feedback[1][0]=.5;
  ASSERT_EQ(c.update(rclcpp::Time(1001000000LL),period),controller_interface::return_type::OK);
  EXPECT_EQ(output,previous);
  ASSERT_EQ(executor.spin_until_future_complete(result, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(send_goal(),nullptr);
  holding=0;feedback[1][0]=.7;
  ASSERT_EQ(c.update(rclcpp::Time(1002000000LL),period),controller_interface::return_type::OK);
  EXPECT_EQ(output,previous);
  EXPECT_EQ(send_goal(),nullptr);
  c.on_deactivate(rclcpp_lifecycle::State{});
  ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State{}),controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(c.update(rclcpp::Time(1003000000LL),period),controller_interface::return_type::OK);
  EXPECT_NE(output,previous);
  for(double value:output) EXPECT_TRUE(std::isfinite(value));
  c.on_deactivate(rclcpp_lifecycle::State{});c.release_interfaces();
}
