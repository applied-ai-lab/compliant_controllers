/**
 * \file task_space_compliant_controller.cpp
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

#include "compliant_controllers/joint_task_space/controller.h"

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
#include <stdexcept>

#include "compliant_controllers/extended_joint_positions.h"
#include "compliant_controllers/robot_state.h"
#include "compliant_controllers/utils.h"


namespace compliant_controllers {
  namespace joint_task_space {

    CompliantController::CompliantController(
        std::unique_ptr<pinocchio::Model> robot_model, std::string const& end_effector_link,
        int const num_controlled_dofs, bool apply_gravity)
    : robot_model_{std::move(robot_model)}, end_effector_link_{end_effector_link},
      data_{std::make_unique<pinocchio::Data>(*robot_model_.get())},
      num_controlled_dofs_{num_controlled_dofs}, apply_gravity_{apply_gravity}
      {

      if (robot_model_->nv != num_controlled_dofs_ ||
          robot_model_->joints.size() != static_cast<std::size_t>(num_controlled_dofs_ + 1))
        throw std::invalid_argument("controller needs a reduced model with only its arm joints");
      if (!robot_model_->existFrame(end_effector_link))
        throw std::invalid_argument("end effector frame not in model: " + end_effector_link);
      end_effector_index_ = robot_model_->getFrameId(end_effector_link);
      for (pinocchio::JointIndex j = 1; j < robot_model_->joints.size(); ++j)
        continuous_joints_.push_back(robot_model_->nqs[j] == 2);
      setDefaultValues();
      extended_joints_ = std::make_unique<ExtendedJointPositions>(num_controlled_dofs_, continuous_joints_);
      // TODO: Potentially pre-allocate also other structures
      return;
    }

    void CompliantController::setDefaultValues() {
      Eigen::MatrixXd const joint_stiffness_matrix {constructDiagonalMatrix(3750, num_controlled_dofs_)};
      inverse_joint_stiffness_matrix_ = joint_stiffness_matrix.inverse();
      rotor_inertia_matrix_ = constructDiagonalMatrix(0.3, num_controlled_dofs_);
      inverse_rotor_inertia_matrix_ = rotor_inertia_matrix_.inverse();
      friction_l_ = constructDiagonalMatrix(60, num_controlled_dofs_);
      friction_lp_ = constructDiagonalMatrix(4, num_controlled_dofs_);
      friction_li_ = constructDiagonalMatrix(0, num_controlled_dofs_);
      joint_k_matrix_ = constructDiagonalMatrix(1, num_controlled_dofs_);
      task_k_matrix_ = constructDiagonalMatrix(10, 6);
      task_d_matrix_ = constructDiagonalMatrix(0.1, 6);
      joint_d_matrix_ = constructDiagonalMatrix(2, num_controlled_dofs_);
      q_error_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      q_error_sum_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      q_error_max_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      tempMat3d_ = Eigen::Matrix3d::Zero();
      tempIsometry3d_ = Eigen::Isometry3d::Identity();
      jacobian_ = Eigen::MatrixXd::Zero(6, robot_model_->nv);
      pinocchio_positions_.setZero(robot_model_->nq);
      for (auto* v : {&last_desired_positions_, &nominal_theta_prev_, &nominal_theta_dot_prev_,
          &desired_positions_, &current_theta_, &gravity_, &task_effort_, &nominal_theta_d_dot_,
          &nominal_theta_dot_, &nominal_theta_, &nominal_friction_, &efforts_}) v->setZero(num_controlled_dofs_);
      return;
    }

    bool CompliantController::setJointStiffnessMatrix(
        Eigen::MatrixXd const& joint_stiffness_matrix) {
      if (!checkMatrix(joint_stiffness_matrix) || !joint_stiffness_matrix.isDiagonal() ||
          (joint_stiffness_matrix.diagonal().array() <= 0).any()) {
        return false;
      }
      inverse_joint_stiffness_matrix_ = joint_stiffness_matrix.inverse();
      return true;
    }

    bool CompliantController::setRotorInertiaMatrix(
        Eigen::MatrixXd const& rotor_inertia_matrix) {
      if (!checkMatrix(rotor_inertia_matrix) || !rotor_inertia_matrix.isDiagonal() ||
          (rotor_inertia_matrix.diagonal().array() <= 0).any()) {
        return false;
      }
      rotor_inertia_matrix_ = rotor_inertia_matrix;
      inverse_rotor_inertia_matrix_ = rotor_inertia_matrix.inverse();
      return true;
    }

    bool CompliantController::setFrictionL(Eigen::MatrixXd const& friction_l) {
      if (!checkMatrix(friction_l)) {
        return false;
      }
      friction_l_ = friction_l;
      return true;
    }

    bool CompliantController::setFrictionLp(Eigen::MatrixXd const& friction_lp) {
      if (!checkMatrix(friction_lp)) {
        return false;
      }
      friction_lp_ = friction_lp;
      return true;
    }

    bool CompliantController::setFrictionLi(Eigen::MatrixXd const& friction_li) {
      if (!checkMatrix(friction_li)) {
        return false;
      }
      friction_li_ = friction_li;
      return true;
    }

    bool CompliantController::setTaskKMatrix(Eigen::MatrixXd const& task_k_matrix) {
      if (!task_k_matrix.allFinite() || (task_k_matrix.rows() != 6) || (task_k_matrix.cols() != 6)) {
        return false;
      }
      task_k_matrix_ = task_k_matrix;
      return true;
    }

    bool CompliantController::setTaskDMatrix(Eigen::MatrixXd const& task_d_matrix)
    {
      if (!task_d_matrix.allFinite() || (task_d_matrix.rows() != 6) || (task_d_matrix.cols() != 6)) {
        return false;
      }
      task_d_matrix_ = task_d_matrix;
      return true;
    }

    bool CompliantController::setJointKMatrix(Eigen::MatrixXd const& joint_k_matrix) {
      if (!checkMatrix(joint_k_matrix)) {
        return false;
      }
      joint_k_matrix_ = joint_k_matrix;
      return true;
    }

    bool CompliantController::setJointDMatrix(Eigen::MatrixXd const& joint_d_matrix) {
      if (!checkMatrix(joint_d_matrix)) {
        return false;
      }
      joint_d_matrix_ = joint_d_matrix;
      return true;
    }

    bool CompliantController::setMaxJointError(Eigen::VectorXd const& joint_error_max)
    {
      if (joint_error_max.size() != num_controlled_dofs_ || !joint_error_max.allFinite() ||
          (joint_error_max.array() < 0).any()) return false;
      q_error_max_ = joint_error_max;
      return true;
    }

    bool CompliantController::init() {
      extended_joints_->reset();
      count_ = 0;
      q_error_.setZero();
      q_error_sum_.setZero();
      return true;
    }

    void CompliantController::advanceKinematics(Eigen::VectorXd const& q)
    {
      joint_ros_to_pinocchio(q, *robot_model_, pinocchio_positions_);
      pinocchio::forwardKinematics(*robot_model_, *data_, pinocchio_positions_);
      pinocchio::computeJointJacobians(*robot_model_, *data_, pinocchio_positions_);
      pinocchio::updateFramePlacements(*robot_model_, *data_);
    }

    Eigen::Isometry3d CompliantController::getFrameTransform(Eigen::VectorXd const& q,
                                                                      pinocchio::Model::Index const& frame_idx)
    {
      advanceKinematics(q);
      tempIsometry3d_ = data_->oMf[frame_idx].toHomogeneousMatrix_impl();
      tempMat3d_ = tempIsometry3d_.linear();
      // [Emprise Lab] Make first and second column negative to account for axis convention
      tempMat3d_.col(0) *= -1.0;
      tempMat3d_.col(1) *= -1.0;
      tempIsometry3d_.linear() = tempMat3d_;
      return tempIsometry3d_;
    }

    const Eigen::VectorXd& CompliantController::computeEffort(RobotState const& desired_state,
        RobotState const&  current_state, double period) {
      if (!std::isfinite(period) || period <= 0.0)
        throw std::invalid_argument("period must be finite and positive");
      for (const auto* state : {&desired_state, &current_state}) {
        if (state->positions.size() != num_controlled_dofs_ ||
            state->velocities.size() != num_controlled_dofs_ ||
            state->efforts.size() != num_controlled_dofs_ ||
            !state->positions.allFinite() || !state->velocities.allFinite() || !state->efforts.allFinite())
          throw std::invalid_argument("joint state dimensions/finite values");
      }
      desired_positions_ = desired_state.positions;

      if (!extended_joints_->isInitialized()) {
        last_desired_positions_ = desired_state.positions;
        [[maybe_unused]] bool const is_success {extended_joints_->init(current_state.positions)};
        extended_joints_->update(current_state.positions);
        nominal_theta_prev_ = extended_joints_->getPositions();
        // nominal_theta_prev_ = current_state.positions;
        nominal_theta_dot_prev_ = current_state.velocities;
        desired_positions_ = nominal_theta_prev_;
      }

      extended_joints_->update(current_state.positions);
      current_theta_ = extended_joints_->getPositions();

      // GELLO sends wrapped continuous angles. Lift the reference onto the measured turn;
      // bounded joints retain their absolute coordinates.
      for (int i = 0; i < num_controlled_dofs_; ++i) {
        if (continuous_joints_[i])
          desired_positions_(i) = current_theta_(i) +
            std::remainder(desired_positions_(i) - current_theta_(i), 2.0 * M_PI);
      }
      try {
        gravity_ = computeGravity(current_theta_, *robot_model_, *data_, pinocchio_positions_);
      } catch (...) {
        nan_pinocchio_throws_.fetch_add(1, std::memory_order_relaxed);
        throw;
      }
      if (!gravity_.allFinite()) {
        nan_pinocchio_output_.fetch_add(1, std::memory_order_relaxed);
        throw std::runtime_error("non-finite model gravity");
      }

      // DESIRED AND NOMINAL TRANSFORMS FOR ERROR PREDICTION

      // Get desired ee position from desired joint states
      desired_ee_transform_ = getFrameTransform(desired_positions_, end_effector_index_);
      desired_ee_quat_ = Eigen::Quaterniond(desired_ee_transform_.linear());

      // Get nominal ee position from desired joint states
      nominal_ee_transform_ = getFrameTransform(nominal_theta_prev_, end_effector_index_);
      nominal_ee_quat_ = Eigen::Quaterniond(nominal_ee_transform_.linear());

      pinocchio::getFrameJacobian(*robot_model_, *data_, end_effector_index_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);
      
      // Reorientate quat if needed
      if (desired_ee_quat_.coeffs().dot(nominal_ee_quat_.coeffs()) < 0.0)
      {
        nominal_ee_quat_.coeffs() = -nominal_ee_quat_.coeffs();
      }

      // Taskspace orientation error
      error_quat_ = nominal_ee_quat_.inverse() * desired_ee_quat_;
      // Update the task space error
      taskspace_error_(3) = error_quat_.x();
      taskspace_error_(4) = error_quat_.y(); 
      taskspace_error_(5) = error_quat_.z();
      taskspace_error_.tail(3) = -nominal_ee_transform_.linear() * taskspace_error_.tail(3);
      // Taskspace position error
      taskspace_error_.head(3) = nominal_ee_transform_.translation() - desired_ee_transform_.translation(); // positional error

      // Gravity is added exactly once below when apply_gravity_ is true.
      // The jacobian includes derivatives for the gripper which does not affect the manipulator so these are ignored
      task_effort_.noalias() = jacobian_.block(0, 0, 6, num_controlled_dofs_).transpose() * (-task_k_matrix_ * taskspace_error_
                                                                                            -task_d_matrix_ * jacobian_.block(0, 0, 6, num_controlled_dofs_) * (nominal_theta_dot_prev_ - desired_state.velocities))
                              - joint_k_matrix_*(nominal_theta_prev_ - desired_positions_ - inverse_joint_stiffness_matrix_*gravity_)
                              - joint_d_matrix_*(nominal_theta_dot_prev_ - desired_state.velocities);

      // Preserve the ROS1 nominal integrator at 1 kHz (enforced by the ROS2 wrapper).
      double const step_time {0.001};
      // Exclaim exports ROS-sign torque; the ROS1 backend exposed its negative.
      nominal_theta_d_dot_.noalias() = inverse_rotor_inertia_matrix_*(task_effort_ + gravity_ - current_state.efforts);
      nominal_theta_dot_ = nominal_theta_dot_prev_ + nominal_theta_d_dot_*step_time;
      nominal_theta_ = nominal_theta_prev_ + nominal_theta_dot_*step_time;

      q_error_sum_ = integrate_error(nominal_theta_, current_theta_);

      nominal_friction_.noalias() = rotor_inertia_matrix_*friction_l_*((nominal_theta_dot_ - current_state.velocities) + 
                          friction_lp_*(nominal_theta_ - current_theta_)
                          + friction_li_ * q_error_sum_);
      
      efforts_ = task_effort_ + nominal_friction_;

      // Apply gravity compensation if desired
      if (apply_gravity_) { efforts_ += gravity_; }

      // F1: Integrator trap-break — see docs/diagnosis_report.md.
      if (!nominal_theta_.allFinite() || !nominal_theta_dot_.allFinite()) {
        nan_integrator_resets_.fetch_add(1, std::memory_order_relaxed);
        nominal_theta_prev_     = current_theta_;
        nominal_theta_dot_prev_ = current_state.velocities;
        throw std::runtime_error("nominal integrator became non-finite");
      } else {
        nominal_theta_prev_ = nominal_theta_;
        nominal_theta_dot_prev_ = nominal_theta_dot_;
      }

      if (!efforts_.allFinite()) throw std::runtime_error("non-finite effort command");
      return efforts_;
    }

    Eigen::VectorXd CompliantController::integrate_error(Eigen::VectorXd const& desired_q,
                                                                  Eigen::VectorXd const& current_q)
    {
      q_error_ = desired_q - current_q;
      q_error_sum_ += q_error_;

      // F3: q_error_sum_ trap-break — cwiseMin/cwiseMax don't recover
      // from NaN.  See docs/diagnosis_report.md C3, F3.
      if (!q_error_sum_.allFinite()) {
        nan_q_error_sum_resets_.fetch_add(1, std::memory_order_relaxed);
        q_error_sum_.setZero();
      }

      // Clamp the integrated error
      q_error_sum_ = q_error_sum_.cwiseMin(q_error_max_).cwiseMax(-q_error_max_);
      return q_error_sum_;
    }

    Eigen::MatrixXd CompliantController::constructDiagonalMatrix(double const value, int const dim) {
      Eigen::VectorXd diag_elements {Eigen::VectorXd::Zero(dim)};
      for (Eigen::Index i = 0; i < diag_elements.size(); ++i) {
        diag_elements[i] = value;
      }
      Eigen::MatrixXd matrix {Eigen::MatrixXd::Zero(dim, dim)};
      matrix.diagonal() = diag_elements;
      return matrix;
    }

    bool CompliantController::checkMatrix(Eigen::MatrixXd const& matrix) const noexcept {
      if (!matrix.allFinite() || (matrix.rows() != num_controlled_dofs_) || (matrix.cols() != num_controlled_dofs_)) {
        return false;
      }
      return true;
    }
  } 
}
