/**
 * \file joint_space_compliant_controller.h
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

#ifndef COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER
#define COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER
#pragma once

#include <chrono>
#include <memory>
#include <string>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>

#include <Eigen/Eigen>
#include <ros/ros.h>

#include "compliant_controllers/extended_joint_positions.h"
#include "compliant_controllers/robot_state.h"


namespace compliant_controllers {

  /**\class JointSpaceCompliantController
   * \brief
   *   Controller that is compliant in joint space
  */
  class JointSpaceCompliantController {
    public:
      /**\fn JointSpaceCompliantController
       * \brief
       *   Constructor that allocates all necessary structures
       * 
       * \param[in] robot_model
       *   The robot model
       * \param[in] end_effector_link
       *   The end-effector link of the kinematic chain
      */
      JointSpaceCompliantController(std::unique_ptr<pinocchio::Model> robot_model,
                                    std::string const& end_effector_link,
                                    int num_controlled_dofs);
      JointSpaceCompliantController() = delete;
      JointSpaceCompliantController(JointSpaceCompliantController const&) = default;
      JointSpaceCompliantController& operator= (JointSpaceCompliantController const&) = default;
      JointSpaceCompliantController(JointSpaceCompliantController&&) = default;
      JointSpaceCompliantController& operator= (JointSpaceCompliantController&&) = default;

      /**\fn setDefaultValues
       * \brief
       *   Set default values to all gains and friction parameters
      */
      void setDefaultValues();

      /**\fn setJointStiffnessMatrix
       * \brief
       *   Set the joint stiffness matrix Ks
       * 
       * \param[in] joint_stiffness_matrix
       *   The joint stiffness matrix to be set
       * \return
       *   Boolean variable signalling success or failure
      */
      [[nodiscard]]
      bool setJointStiffnessMatrix(Eigen::MatrixXd const& joint_stiffness_matrix);

      /**\fn setRotorInertiaMatrix
       * \brief
       *   Set the rotor inertia matrix Kr
       * 
       * \param[in] rotor_inertia_matrix
       *   The rotor inertia matrix to be set
       * \return
       *   Boolean variable signalling success or failure
      */
      [[nodiscard]]
      bool setRotorInertiaMatrix(Eigen::MatrixXd const& rotor_inertia_matrix);

      /**\fn setFrictionL
       * \brief
       *   Set the friction observer matrix 1
       * 
       * \param[in] friction_l
       *   The friction observer matrix 1 to be set
       * \return
       *   Boolean variable signalling success or failure
      */
      [[nodiscard]]
      bool setFrictionL(Eigen::MatrixXd const& friction_l);

      /**\fn setFrictionLp
       * \brief
       *   Set the friction observer matrix 2
       * 
       * \param[in] friction_lp
       *   The friction observer matrix 2 to be set
       * \return
       *   Boolean variable signalling success or failure
      */
      [[nodiscard]]
      bool setFrictionLp(Eigen::MatrixXd const& friction_lp);

      /**\fn setJointKMatrix
       * \brief
       *   Set the joint compliance proportional gain matrix
       * 
       * \param[in] joint_k_matrix
       *   The joint compliance proportional gain matrix to be set
       * \return
       *   Boolean variable signalling success or failure
      */
      [[nodiscard]]
      bool setJointKMatrix(Eigen::MatrixXd const& joint_k_matrix);

      /**\fn setJointDMatrix
       * \brief
       *   Set the joint compliance derivative gain matrix
       * 
       * \param[in] joint_d_matrix
       *   The joint compliance derivative gain matrix to be set
       * \return
       *   Boolean variable signalling success or failure
      */
      [[nodiscard]]
      bool setJointDMatrix(Eigen::MatrixXd const& joint_d_matrix);

      /**\fn init
       * \brief
       *   Initialize the controller
       * 
       * \return
       *   Boolean variable signalling success or failure
      */
      [[nodiscard]]
      bool init();

      /**\fn computeEffort
       * \brief
       *   Compute the required effort for bringing the robot from state \p current_state to state \p desired_state
       * 
       * \param[in] desired_state
       *   The state that the robot should be set to
       * \param[in] current_state
       *   The state that the robot is currently in
       * \param[in] period
       *   The period of the controller
       * \return
       *   The effort that should be applied to the individual joints to move to the desired state
      */
      [[nodiscard]]
      Eigen::VectorXd computeEffort(RobotState const& desired_state, RobotState const& current_state, ros::Duration const& period);

    protected:
      /**\fn constructDiagonalMatrix
       * \brief
       *   Construct a diagonal matrix of dimension \p dim with \p value in its diagonal
       *
       * \param[in] value
       *   The value to be set to the diagonal
       * \param[in] dim
       *   The dimension of the matrix
       * \return
       *   The matrix of dimension \p dim and \p value in its diagonal
      */
      [[nodiscard]]
      static Eigen::MatrixXd constructDiagonalMatrix(double const value, int const dim);

      /**\fn checkMatrix
       * \brief
       *   Check if the dimensions of the matrix match with the expected dimensions
       *
       * \param[in] matrix
       *   The matrix to be checked
       * \return
       *   Boolean value signalling same (true) or different dimensions (false)
      */
      [[nodiscard]]
      bool checkMatrix(Eigen::MatrixXd const& matrix) const noexcept;

      std::unique_ptr<pinocchio::Model> robot_model_;
      std::string end_effector_link_;
      std::unique_ptr<pinocchio::Data> data_;
      int num_controlled_dofs_;
      pinocchio::Model::Index end_effector_index_;
      Eigen::MatrixXd joint_stiffness_matrix_;
      Eigen::MatrixXd rotor_inertia_matrix_;
      Eigen::MatrixXd friction_l_;
      Eigen::MatrixXd friction_lp_;
      Eigen::MatrixXd joint_k_matrix_;
      Eigen::MatrixXd joint_d_matrix_;
      std::unique_ptr<ExtendedJointPositions> extended_joints_;
      int count_;

      Eigen::VectorXd last_desired_positions_;
      Eigen::VectorXd nominal_theta_prev_;
      Eigen::VectorXd nominal_theta_dot_prev_;
      Eigen::VectorXd desired_positions_;
      Eigen::VectorXd current_theta_;
      Eigen::VectorXd gravity_;
      Eigen::VectorXd desired_theta_;
      Eigen::VectorXd desired_theta_dot_;
      Eigen::VectorXd task_effort_;
      Eigen::VectorXd nominal_theta_d_dot_;
      Eigen::VectorXd nominal_theta_dot_;
      Eigen::VectorXd nominal_theta_;
      Eigen::VectorXd nominal_friction_;
      Eigen::VectorXd efforts_;
  };

}

#endif // COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER