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
  namespace joint_space {

    /**\class JointSpaceCompliantController
     * \brief
     *   Controller that is compliant in joint space
    */
    class CompliantController {
      public:
        /**\fn JointSpaceCompliantController
         * \brief
         *   Constructor that allocates all necessary structures
         * 
         * \param[in] robot_model
         *   The robot model
         * \param[in] end_effector_link
         *   The end-effector link of the kinematic chain
         * \param[in] num_controlled_dofs
         *   The number of degree of freedoms that the controller should control
         * \param[in] apply_gravity
         *   Toggle whether to add gravity compensatin to efforts_ during computeEffort
        */
        CompliantController(std::unique_ptr<pinocchio::Model> robot_model,
                                      std::string const& end_effector_link,
                                      int const num_controlled_dofs,
                                      bool apply_gravity);
        CompliantController() = delete;
        CompliantController(CompliantController const&) = default;
        CompliantController& operator= (CompliantController const&) = default;
        CompliantController(CompliantController&&) = default;
        CompliantController& operator= (CompliantController&&) = default;

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

        /**\fn setFrictionLi
         * \brief
         *   Set the friction observer matrix integral gain
         * 
         * \param[in] friction_li
         *   The friction observer matrix integral gain
         * \return
         *   Boolean variable signalling success or failure
        */
        [[nodiscard]]
        bool setFrictionLi(Eigen::MatrixXd const& friction_li);

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

        /**\fn setMaxJointError
         * \brief
         *   Set the joint integrator max error
         * 
         * \param[in] joint_error_max
         *   The joint compliance derivative gain matrix to be set
         * \return
         *   Boolean variable signalling success or failure
        */
        [[nodiscard]]
        bool setMaxJointError(Eigen::VectorXd const& joint_error_max);

        /**\fn init
         * \brief
         *   Initialize and Reset the controller
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

        /**\fn integrate_error
         * \brief
         *   Sum error between \p current_q and \p desired_q clamp using q_error_max_
         * 
         * \param[in] desired_q
         *   The state that the robot is currently in
         * \param[in] current_q
         *   The state that the robot should be set to
         * \return
         *   The sum of the errors
        */
        [[nodiscard]]
        Eigen::VectorXd integrate_error(Eigen::VectorXd const& desired_q, 
                                        Eigen::VectorXd const& current_q);
        

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

        bool apply_gravity_;
        std::unique_ptr<pinocchio::Model> robot_model_;
        std::string end_effector_link_;
        std::unique_ptr<pinocchio::Data> data_;
        int num_controlled_dofs_;
        pinocchio::Model::Index end_effector_index_;
        Eigen::MatrixXd inverse_joint_stiffness_matrix_;
        Eigen::MatrixXd rotor_inertia_matrix_;
        Eigen::MatrixXd inverse_rotor_inertia_matrix_;
        Eigen::MatrixXd friction_l_;
        Eigen::MatrixXd friction_lp_;
        Eigen::MatrixXd friction_li_;           // Integral friction observer gain
        Eigen::MatrixXd joint_k_matrix_;
        Eigen::MatrixXd joint_d_matrix_;
        // Integrator quantities
        Eigen::VectorXd q_error_;
        Eigen::VectorXd q_error_sum_;
        Eigen::VectorXd q_error_max_;
        std::unique_ptr<ExtendedJointPositions> extended_joints_;
        int count_;

        Eigen::VectorXd last_desired_positions_;
        Eigen::VectorXd nominal_theta_prev_;
        Eigen::VectorXd nominal_theta_dot_prev_;
        Eigen::VectorXd desired_positions_;
        Eigen::VectorXd current_theta_;
        Eigen::VectorXd gravity_;
        Eigen::VectorXd task_effort_;
        Eigen::VectorXd nominal_theta_d_dot_;
        Eigen::VectorXd nominal_theta_dot_;
        Eigen::VectorXd nominal_theta_;
        Eigen::VectorXd nominal_friction_;
        Eigen::VectorXd efforts_;
    };
  } // namespace compliant_controllers
} // joint_space

#endif // COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER
