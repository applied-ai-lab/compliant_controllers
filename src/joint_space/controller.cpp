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

#include "compliant_controllers/joint_space/controller.h"

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
  namespace joint_space {

    CompliantController::CompliantController(
        std::unique_ptr<pinocchio::Model> robot_model, std::string const& end_effector_link,
        int const num_controlled_dofs, bool apply_gravity)
    : robot_model_{std::move(robot_model)}, end_effector_link_{},
      data_{std::make_unique<pinocchio::Data>(*robot_model_.get())},
      num_controlled_dofs_{num_controlled_dofs}, apply_gravity_{apply_gravity},
      end_effector_index_{robot_model_->getFrameId(end_effector_link)} {
      setDefaultValues();
      extended_joints_ = std::make_unique<ExtendedJointPositions>(num_controlled_dofs_);
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
      joint_k_matrix_ = constructDiagonalMatrix(10, num_controlled_dofs_);
      joint_d_matrix_ = constructDiagonalMatrix(2, num_controlled_dofs_);
      q_error_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      q_error_sum_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      q_error_max_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      return;
    }

    bool CompliantController::setJointStiffnessMatrix(
        Eigen::MatrixXd const& joint_stiffness_matrix) {
      if (!checkMatrix(joint_stiffness_matrix)) {
        return false;
      }
      inverse_joint_stiffness_matrix_ = joint_stiffness_matrix.inverse();
      return true;
    }

    bool CompliantController::setRotorInertiaMatrix(
        Eigen::MatrixXd const& rotor_inertia_matrix) {
      if (!checkMatrix(rotor_inertia_matrix)) {
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
      if (!joint_error_max.rows() != num_controlled_dofs_) return false;
      q_error_max_ = joint_error_max;
      return true;
    }

    bool CompliantController::init() {
      count_ = 0;
      q_error_.setZero();
      q_error_sum_.setZero();
      return true;
    }

    Eigen::VectorXd CompliantController::computeEffort(RobotState const& desired_state,
        RobotState const&  current_state, ros::Duration const& period) {    
      desired_positions_ = desired_state.positions;

      if (!extended_joints_->isInitialized()) {
        last_desired_positions_ = desired_state.positions;
        [[maybe_unused]] bool const is_success {extended_joints_->init(current_state.positions)};
        extended_joints_->update(current_state.positions);
        nominal_theta_prev_ = extended_joints_->getPositions();
        nominal_theta_dot_prev_ = current_state.velocities;
        desired_positions_ = nominal_theta_prev_;
      }

      extended_joints_->update(current_state.positions);
      current_theta_ = extended_joints_->getPositions();

      // F2: Pinocchio defence.  computeGeneralizedGravity can throw
      // (dimension mismatch, internal asserts) or return a vector
      // with non-finite entries if `q_` contains NaN.  Either failure
      // mode silently poisons the integrator chain below — see
      // docs/diagnosis_report.md C2.  Catch the throw, scan the
      // output, increment the appropriate counter; on any failure
      // zero out the gravity for this cycle.  Combined with F1's
      // integrator trap-break, this guarantees gravity_ is finite
      // before line 170's multiply.
      try {
        gravity_ = pinocchio::computeGeneralizedGravity(
            *robot_model_, *data_,
            joint_ros_to_pinocchio(current_theta_, *robot_model_));
      } catch (const std::exception& ex) {
        nan_pinocchio_throws_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR_THROTTLE(
            1.0,
            "[compliant_controllers/joint_space] Pinocchio "
            "computeGeneralizedGravity threw: %s — zeroing gravity",
            ex.what());
        gravity_ = Eigen::VectorXd::Zero(num_controlled_dofs_);
      }
      if (!gravity_.allFinite()) {
        nan_pinocchio_output_.fetch_add(1, std::memory_order_relaxed);
        ROS_WARN_THROTTLE(
            1.0,
            "[compliant_controllers/joint_space] Pinocchio gravity "
            "non-finite — zeroing affected entries");
        for (Eigen::Index i = 0; i < gravity_.size(); ++i) {
          if (!std::isfinite(gravity_[i])) gravity_[i] = 0.0;
        }
      }

      // Gravity compensation is performed inside the hardware interface
      task_effort_ = -joint_k_matrix_*(nominal_theta_prev_ - desired_positions_ - inverse_joint_stiffness_matrix_*gravity_) - 
                      joint_d_matrix_*(nominal_theta_dot_prev_ - desired_state.velocities);

      // Time step smaller than period.toSec() potentially because of too small rotor inertia matrix
      double const step_time {0.001};
      // Sign of the current state effort seems to be negative of commanded
      nominal_theta_d_dot_ = inverse_rotor_inertia_matrix_*(task_effort_ + gravity_ + current_state.efforts);
      nominal_theta_dot_ = nominal_theta_dot_prev_ + nominal_theta_d_dot_*step_time;
      nominal_theta_ = nominal_theta_prev_ + nominal_theta_dot_*step_time;

      // Integrate friction error
      q_error_sum_ = integrate_error(nominal_theta_, current_theta_);

      nominal_friction_ = rotor_inertia_matrix_*friction_l_*((nominal_theta_dot_ - current_state.velocities) + 
                          friction_lp_*(nominal_theta_ - current_theta_)
                          + friction_li_ * q_error_sum_);

      efforts_ = task_effort_ + nominal_friction_;

      // Apply gravity compensation if required
      if (apply_gravity_) { efforts_ += gravity_; }

      // F1: Integrator trap-break.  `nominal_theta_*_prev_` are the
      // persistent integrator state for the controller — once they
      // become non-finite, every subsequent computeEffort produces
      // non-finite output and no in-loop mechanism recovers (init()
      // doesn't reset them; switch_controller doesn't help; only a
      // full re-construction of CompliantController does — which
      // means a process restart).  Detect a non-finite update HERE,
      // before it poisons the next cycle, and reset to the current
      // measurements (lose one cycle of momentum tracking but break
      // the lock-up).  See docs/diagnosis_report.md F1 + C1.
      if (!nominal_theta_.allFinite() || !nominal_theta_dot_.allFinite()) {
        nan_integrator_resets_.fetch_add(1, std::memory_order_relaxed);
        ROS_WARN_THROTTLE(
            1.0,
            "[compliant_controllers/joint_space] integrator state "
            "went non-finite — resetting nominal_theta_*_prev_ to "
            "current measurements");
        nominal_theta_prev_     = current_theta_;
        nominal_theta_dot_prev_ = current_state.velocities;
        // Also force efforts_ to a safe value (zero) for THIS
        // cycle — the integrator that produced efforts_ was
        // already corrupted, so its output is junk.  The next
        // cycle starts fresh from the reset _prev_ values.
        efforts_.setZero();
      } else {
        nominal_theta_prev_     = nominal_theta_;
        nominal_theta_dot_prev_ = nominal_theta_dot_;
      }

      return efforts_;
    }

    Eigen::VectorXd CompliantController::integrate_error(Eigen::VectorXd const& desired_q,
                                                                  Eigen::VectorXd const& current_q)
    {
      q_error_ = desired_q - current_q;
      q_error_sum_ += q_error_;

      // F3: q_error_sum_ trap-break.  cwiseMin/cwiseMax do NOT
      // recover from NaN per IEEE 754 (min(NaN, finite) == NaN).
      // So a one-cycle NaN in q_error_ leaves q_error_sum_ NaN
      // forever, poisoning the friction term every subsequent
      // cycle.  Detect and zero — sacrifice the integral term for
      // this cycle, but break the trap.  See
      // docs/diagnosis_report.md F3 + C3.
      if (!q_error_sum_.allFinite()) {
        nan_q_error_sum_resets_.fetch_add(1, std::memory_order_relaxed);
        ROS_WARN_THROTTLE(
            1.0,
            "[compliant_controllers/joint_space] q_error_sum_ went "
            "non-finite — resetting to zero");
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
      if ((matrix.rows() != num_controlled_dofs_) || (matrix.cols() != num_controlled_dofs_)) {
        return false;
      }
      return true;
    }

  } // namespace joint_space
} // namespace compliant_controllers