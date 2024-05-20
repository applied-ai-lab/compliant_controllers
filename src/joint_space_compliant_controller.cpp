/**
 * \file joint_space_compliant_controller.cpp
 * \mainpage
 *   Compliant controller in joint space
 * 
 * \authors
 *   Tobit Flatscher <tobit@robots.ox.ac.uk>
 *   Rishabh Madan <rm773@cornell.edu>
 *   Rajat Kumar Jenamani <rj277@cornell.edu>
 * \copyright
 *   Oxford Robotics Institute - University of Oxford (2024)
 *   EmPRISE Lab - Cornell University (2023)
 * \license
 *   This project is released under the 3-clause BSD license.
*/

#include "compliant_controllers/joint_space_compliant_controller.h"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>

#include <Eigen/Eigen>
#include <ros/ros.h>

#include "compliant_controllers/extended_joint_positions.h"
#include "compliant_controllers/robot_state.h"
#include "compliant_controllers/utils.h"


namespace compliant_controllers {

  JointSpaceCompliantController::JointSpaceCompliantController(
      std::unique_ptr<pinocchio::Model> robot_model, std::string const& end_effector_link)
  : robot_model_{std::move(robot_model)}, end_effector_link_{},
    data_{std::make_unique<pinocchio::Data>(*robot_model_.get())},
    num_controlled_dofs_{robot_model_->nv},
    end_effector_index_{robot_model_->getFrameId(end_effector_link)} {
    setDefaultValues();
    extended_joints_ = std::make_unique<ExtendedJointPositions>(num_controlled_dofs_);
    // TODO: Potentially pre-allocate also other structures
    return;
  }

  void JointSpaceCompliantController::setDefaultValues() {
    joint_stiffness_matrix_ = constructDiagonalMatrix(3750, num_controlled_dofs_);
    rotor_inertia_matrix_ = constructDiagonalMatrix(0.3, num_controlled_dofs_);
    friction_l_ = constructDiagonalMatrix(60, num_controlled_dofs_);
    friction_lp_ = constructDiagonalMatrix(4, num_controlled_dofs_);
    joint_k_matrix_ = constructDiagonalMatrix(10, num_controlled_dofs_);
    joint_d_matrix_ = constructDiagonalMatrix(2, num_controlled_dofs_);
    return;
  }

  bool JointSpaceCompliantController::setJointStiffnessMatrix(
      Eigen::MatrixXd const& joint_stiffness_matrix) {
    if (!checkMatrix(joint_stiffness_matrix)) {
      return false;
    }
    joint_stiffness_matrix_ = joint_stiffness_matrix;
    return true;
  }

  bool JointSpaceCompliantController::setRotorInertiaMatrix(
      Eigen::MatrixXd const& rotor_inertia_matrix) {
    if (!checkMatrix(rotor_inertia_matrix)) {
      return false;
    }
    rotor_inertia_matrix_ = rotor_inertia_matrix;
    return true;
  }

  bool JointSpaceCompliantController::setFrictionL(Eigen::MatrixXd const& friction_l) {
    if (!checkMatrix(friction_l)) {
      return false;
    }
    friction_l_ = friction_l;
    return true;
  }

  bool JointSpaceCompliantController::setFrictionLp(Eigen::MatrixXd const& friction_lp) {
    if (!checkMatrix(friction_lp)) {
      return false;
    }
    friction_lp_ = friction_lp;
    return true;
  }

  bool JointSpaceCompliantController::setJointKMatrix(Eigen::MatrixXd const& joint_k_matrix) {
    if (!checkMatrix(joint_k_matrix)) {
      return false;
    }
    joint_k_matrix_ = joint_k_matrix;
    return true;
  }

  bool JointSpaceCompliantController::setJointDMatrix(Eigen::MatrixXd const& joint_d_matrix) {
    if (!checkMatrix(joint_d_matrix)) {
      return false;
    }
    joint_d_matrix_ = joint_d_matrix;
    return true;
  }

  bool JointSpaceCompliantController::init() {
    count_ = 0;
    return true;
  }

  Eigen::VectorXd JointSpaceCompliantController::computeEffort(RobotState const& desired_state,
      RobotState const&  current_state, ros::Duration const& period) {
    desired_positions_ = desired_state.positions;

    if (!extended_joints_->isInitialized()) {
      last_desired_positions_ = desired_state.positions;
      [[maybe_unused]] bool const is_success {extended_joints_->init(current_state.positions)};
      extended_joints_->update(current_state.positions);
      nominal_theta_prev_ = extended_joints_->getPositions();
      nominal_theta_dot_prev_ = current_state.velocities;
      desired_positions_ = extended_joints_->getPositions();
    }

    // TODO: Does this make sense without tolerances?
    if (desired_positions_ != last_desired_positions_ && current_state.positions != desired_positions_) {
      last_desired_positions_ = desired_positions_;
      desired_positions_ = extended_joints_->getPositions();
    }

    extended_joints_->update(current_state.positions);
    current_theta_ = extended_joints_->getPositions();
    gravity_ = pinocchio::computeGeneralizedGravity(*robot_model_, *data_, joint_ros_to_pinocchio(current_theta_, *robot_model_));

    desired_theta_ = desired_positions_ + joint_stiffness_matrix_.inverse()*gravity_;
    desired_theta_dot_ = desired_state.velocities;

    task_effort_ = -joint_k_matrix_*(nominal_theta_prev_ - desired_theta_) - joint_d_matrix_*(nominal_theta_prev_ - desired_theta_dot_);

    double const step_time {period.toSec()};

    nominal_theta_d_dot_ = rotor_inertia_matrix_.inverse()*(task_effort_ + gravity_ + current_state.efforts);
    nominal_theta_dot_ = nominal_theta_dot_prev_ + nominal_theta_d_dot_*step_time;
    nominal_theta_ = nominal_theta_prev_ + nominal_theta_dot_*step_time;

    nominal_theta_prev_ = nominal_theta_;
    nominal_theta_dot_prev_ = nominal_theta_dot_;

    nominal_friction_ = rotor_inertia_matrix_*friction_l_*((nominal_theta_dot_prev_ - current_state.velocities) + friction_lp_*(nominal_theta_prev_ - current_theta_));

    efforts_ = task_effort_ + nominal_friction_;

    // TODO: We will be falling for 50 iterations before we generate any effort, won't we?
    if (count_ < 50) {
      efforts_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      ++count_;
    }
    return efforts_;
  }

  Eigen::MatrixXd JointSpaceCompliantController::constructDiagonalMatrix(double const value, int const dim) {
    Eigen::VectorXd diag_elements {Eigen::VectorXd::Zero(dim)};
    for (Eigen::Index i = 0; i < diag_elements.size(); ++i) {
      diag_elements[i] = value;
    }
    Eigen::MatrixXd matrix {Eigen::MatrixXd::Zero(dim, dim)};
    matrix.diagonal() = diag_elements;
    return matrix;
  }

  bool JointSpaceCompliantController::checkMatrix(Eigen::MatrixXd const& matrix) const noexcept {
    if ((matrix.rows() != num_controlled_dofs_) || (matrix.cols() != num_controlled_dofs_)) {
      return false;
    }
    return true;
  }

}