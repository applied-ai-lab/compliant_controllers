#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <limits>
#include <pinocchio/algorithm/rnea.hpp>
#include "compliant_controllers/model.hpp"
#include "compliant_controllers/utils.h"
#include "compliant_controllers/joint_task_space/controller.h"
using namespace compliant_controllers;
using Core = joint_task_space::CompliantController;
namespace {
const std::string tool = "right_kinova_arm_tool_frame";
std::vector<std::string> names() {
  std::vector<std::string> n;
  for (int i=1;i<=7;++i) n.push_back("right_kinova_arm_joint_"+std::to_string(i));
  return n;
}
std::string xml() {
  std::ifstream f(std::string(TEST_SOURCE_DIR)+"/urdf/oxf20_right_arm.urdf");
  return {std::istreambuf_iterator<char>(f),{}};
}
std::unique_ptr<Core> core(bool gravity=true) {
  auto model = reduceToArm(xml(), names(), tool);
  return std::make_unique<Core>(std::move(model.model), tool, 7, gravity);
}
}
TEST(Core, HistoricalRos1TraceWithRosTorqueSigns) {
  auto c=core();
  ASSERT_TRUE(c->init());
  std::ifstream file(std::string(TEST_SOURCE_DIR)+"/ros2/legacy_trace.csv");
  ASSERT_TRUE(file.good());
  std::string line; std::size_t rows=0;
  while(std::getline(file,line)) {
    if(line.empty() || line[0]=='#') continue;
    std::replace(line.begin(),line.end(),',',' ');
    std::istringstream stream(line);
    RobotState d(7), q(7); Eigen::VectorXd expected(7);
    for(auto* v : {&d.positions,&d.velocities,&q.positions,&q.velocities,&q.efforts,&expected})
      for(int j=0;j<7;++j) ASSERT_TRUE(bool(stream>>(*v)[j]));
    const auto actual=c->computeEffort(d,q,.001);
    ASSERT_TRUE(actual.isApprox(expected,1e-10)) << "row " << rows << "\nactual " << actual.transpose() << "\nexpected " << expected.transpose();
    ++rows;
  }
  EXPECT_EQ(rows,300u);
}
TEST(Core, GravityAddedExactlyOnceWithRosSignFeedback) {
  auto model=reduceToArm(xml(),names(),tool);
  RobotState q(7), d(7); q.positions.setConstant(.4); d.positions=q.positions;
  pinocchio::Data data(*model.model);
  q.efforts=pinocchio::computeGeneralizedGravity(*model.model,data,joint_ros_to_pinocchio(q.positions,*model.model));
  auto with=core(true), without=core(false);
  const auto a=with->computeEffort(d,q,.001), b=without->computeEffort(d,q,.001);
  EXPECT_TRUE((a-b).isApprox(q.efforts,1e-12));
}
TEST(Core, ReactivationReseedsAllNominalAndUnwrappedState) {
  auto used=core(), fresh=core(); RobotState q(7), d(7);
  q.positions.setConstant(.4); d.positions.setConstant(.5);
  for(int i=0;i<100;++i) (void)used->computeEffort(d,q,.001);
  ASSERT_TRUE(used->init());
  q.positions.setConstant(1.1); d.positions=q.positions;
  EXPECT_TRUE(used->computeEffort(d,q,.001).isApprox(fresh->computeEffort(d,q,.001),1e-12));
}
TEST(Core, ContinuousReferenceSeamUsesNearestPhysicalTurn) {
  auto a=core(), b=core(); RobotState q(7), d(7);
  q.positions[0]=3.13; d.positions=q.positions;
  (void)a->computeEffort(d,q,.001); (void)b->computeEffort(d,q,.001);
  d.positions[0]=-3.13;
  const auto torque=a->computeEffort(d,q,.001);
  d.positions[0]+=2*M_PI;
  EXPECT_TRUE(torque.isApprox(b->computeEffort(d,q,.001),1e-12));
}
TEST(Core, RejectsInvalidStateAndSingularInertia) {
  auto c=core(); RobotState q(7),d(7);
  EXPECT_FALSE(c->setRotorInertiaMatrix(Eigen::MatrixXd::Zero(7,7)));
  EXPECT_FALSE(c->setJointStiffnessMatrix(Eigen::MatrixXd::Zero(7,7)));
  EXPECT_TRUE(c->setMaxJointError(Eigen::VectorXd::Constant(7,.5)));
  EXPECT_FALSE(c->setMaxJointError(Eigen::VectorXd::Zero(6)));
  q.efforts[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)c->computeEffort(d,q,.001),std::invalid_argument);
  q.efforts.setZero(); q.positions.resize(6);
  EXPECT_THROW((void)c->computeEffort(d,q,.001),std::invalid_argument);
}
TEST(Model, MapsConfiguredJointOrderAndRejectsMissingJoints) {
  auto n=names(); std::reverse(n.begin(),n.end());
  auto m=reduceToArm(xml(),n,tool);
  for(int i=0;i<7;++i) EXPECT_EQ(m.indices[i],6u-i);
  n[0]=n[1]; EXPECT_THROW(reduceToArm(xml(),n,tool),std::invalid_argument);
}
TEST(Unwrap, BoundedJointsRemainAbsoluteAndResetCanReanchor) {
  ExtendedJointPositions q(2,{true,false});
  Eigen::Vector2d first(3.13,3.5), next(-3.13,-3.5);
  ASSERT_TRUE(q.init(first)); q.update(next);
  EXPECT_NEAR(q.getPositions()[0],2*M_PI-3.13,1e-12);
  EXPECT_DOUBLE_EQ(q.getPositions()[1],-3.5);
  q.reset(); ASSERT_TRUE(q.init(Eigen::Vector2d(.7,.8)));
  EXPECT_DOUBLE_EQ(q.getPositions()[0],.7);
}
