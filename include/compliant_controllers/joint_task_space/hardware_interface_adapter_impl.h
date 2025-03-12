/**
 * \file hardware_interface_adapter_impl.h
 * \mainpage
 *   Implementation of the compliant hardware interface adapter
 * 
 * \authors
 *   Alexander Mitchell <mitch@robots.ox.ac.uk>
 *   Tobit Flatscher <tobit@robots.ox.ac.uk>
 * \copyright
 *   Oxford Robotics Institute - University of Oxford (2024)
 * \license
 *   This project is released under the 3-clause BSD license.
*/

#ifndef COMPLIANT_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER_IMPL
#define COMPLIANT_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER_IMPL
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/fwd.hpp>

#include <dynamic_reconfigure/server.h>
#include <Eigen/Eigen>
#include <hardware_interface/joint_command_interface.h>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <ros/ros.h>

#include "compliant_controllers/JointTaskSpaceCompliantControllerConfig.h"
#include "compliant_controllers/joint_task_space/controller.h"
#include "compliant_controllers/robot_state.h"


namespace compliant_controllers {
  namespace joint_task_space {

    template <typename State>
    CompliantHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State>::CompliantHardwareInterfaceAdapter()
    : joint_handles_ptr_{nullptr} {
      return;
    }

    template <typename State>
    bool CompliantHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State>::init(
        std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh) {
      joint_handles_ptr_ = &joint_handles;

      std::string robot_description_parameter {"/robot_description"};
      if (!controller_nh.getParam("robot_description_parameter", robot_description_parameter)) {
        ROS_WARN_STREAM("Failed getting robot description parameter, defaulting to '" << robot_description_parameter << "'!");
      }
      std::string robot_description {};
      if (!controller_nh.getParam(robot_description_parameter, robot_description)) {
        ROS_ERROR_STREAM("Failed getting robot description from '" << robot_description_parameter << "'!");
        return false;
      }
      auto robot_model = std::make_unique<pinocchio::Model>();
      pinocchio::urdf::buildModelFromXML(robot_description, *robot_model.get());
      if (!robot_model) {
        ROS_ERROR_STREAM("Failed to load Pinocchio model from robot description '" << robot_description << "'!");
        return false;
      }
      std::string end_effector_link {};
      if (!controller_nh.getParam("end_effector_link", end_effector_link)) {
        ROS_ERROR_STREAM("Failed to get robot end effector link!");
        return false;
      }

      num_of_dof_ = joint_handles_ptr_->size();
      desired_state_ = RobotState(num_of_dof_);
      current_state_ = RobotState(num_of_dof_);
      command_effort_.resize(num_of_dof_);

      // TODO: This should be templated so that we can switch between the joint and task space implementations easily
      compliant_controller_ = std::make_unique<CompliantController>(std::move(robot_model), end_effector_link, num_of_dof_);

      using namespace boost::placeholders;
      dynamic_reconfigure_callback_ = boost::bind(&CompliantHardwareInterfaceAdapter::dynamicReconfigureCallback, this, _1, _2);
      dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
      return true;
    }

    template <typename State>
    void CompliantHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State>::starting(
        ros::Time const& /*time*/) {
      if (!joint_handles_ptr_) {
        ROS_ERROR_STREAM("Joint handles not initialized!");
        return;
      }
      execute_default_command_ = true;
      bool is_success {compliant_controller_->init()};
      return;
    }

    template <typename State>
    void CompliantHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State>::stopping(
        ros::Time const& /*time*/) {
      return;
    }

    template <typename State>
    void CompliantHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State>::updateCommand(
        ros::Time const& /*time*/, ros::Duration const& period, State const& desired_state,
        State const& state_error) {
      if (!joint_handles_ptr_) {
        ROS_ERROR_STREAM("Joint handles not initialized!");
        return;
      }

      for (std::size_t i = 0; i < joint_handles_ptr_->size(); ++i) {
        auto& joint_handle {(*joint_handles_ptr_)[i]};
        current_state_.positions(i) = joint_handle.getPosition();
        if (current_state_.positions(i) < 0.0) {
          current_state_.positions(i) += 2.0*M_PI;
        }
        current_state_.velocities(i) = joint_handle.getVelocity();
        current_state_.efforts(i) = joint_handle.getEffort();
      }
      // TODO: Implement a compile-time type check for the State template parameter
      if (!execute_default_command_) {
        for (std::size_t i = 0; i < desired_state.position.size(); ++i) {
          desired_state_.positions(i) = desired_state.position[i];
          desired_state_.velocities(i) = desired_state.velocity[i];
          desired_state_.accelerations(i) = desired_state.acceleration[i];
        }
      } else {
        desired_state_.positions = current_state_.positions;
        desired_state_.velocities.setZero();
        execute_default_command_ = false;
      }

      // TODO: Perform checks that the dimensions are correct!
      command_effort_ = compliant_controller_->computeEffort(desired_state_, current_state_, period);

      for (Eigen::Index i = 0; i < command_effort_.size(); ++i) {
        (*joint_handles_ptr_)[i].setCommand(command_effort_(i));
      }
      return;
    }

    template <typename State>
    void CompliantHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, 
      State>::dynamicReconfigureCallback(JointTaskSpaceCompliantControllerConfig const& config,
          [[maybe_unused]] uint32_t const level) {
      Eigen::MatrixXd joint_stiffness_matrix {Eigen::MatrixXd::Zero(7,7)};
      joint_stiffness_matrix.diagonal() << config.j_0, config.j_1, config.j_2, 
                                          config.j_3, config.j_4, config.j_5, config.j_6;
      [[maybe_unused]] bool is_success {compliant_controller_->setJointStiffnessMatrix(
        joint_stiffness_matrix.block(0, 0, num_of_dof_, num_of_dof_)
      )};

      Eigen::MatrixXd rotor_inertia_matrix {Eigen::MatrixXd::Zero(7,7)};
      rotor_inertia_matrix.diagonal() << config.b_0, config.b_1, config.b_2, 
                                        config.b_3, config.b_4, config.b_5, config.b_6;
      is_success = compliant_controller_->setRotorInertiaMatrix(
        rotor_inertia_matrix.block(0, 0, num_of_dof_, num_of_dof_)
      );

      Eigen::MatrixXd friction_l {Eigen::MatrixXd::Zero(7,7)};
      friction_l.diagonal() << config.l_0, config.l_1, config.l_2,
                              config.l_3, config.l_4, config.l_5, config.l_6;
      is_success = compliant_controller_->setFrictionL(
        friction_l.block(0, 0, num_of_dof_, num_of_dof_)
      );

      Eigen::MatrixXd friction_lp {Eigen::MatrixXd::Zero(7,7)};
      friction_lp.diagonal() << config.lp_0, config.lp_1, config.lp_2,
                                config.lp_3, config.lp_4, config.lp_5, config.lp_6;
      is_success = compliant_controller_->setFrictionLp(
        friction_lp.block(0, 0, num_of_dof_, num_of_dof_)
      );

      Eigen::MatrixXd friction_li {Eigen::MatrixXd::Zero(7,7)};
      friction_li.diagonal() << config.li_0, config.li_1, config.li_2,
                                config.li_3, config.li_4, config.li_5, config.li_6;
      is_success = compliant_controller_->setFrictionLi(
        friction_li.block(0, 0, num_of_dof_, num_of_dof_)
      );

      Eigen::VectorXd friction_e_max {Eigen::VectorXd::Zero(7)};
      friction_e_max << config.e_max_0, config.e_max_1, config.e_max_2, 
                        config.e_max_3, config.e_max_4, config.e_max_5, config.e_max_6;
      is_success = compliant_controller_->setMaxJointError(
        friction_e_max.head(num_of_dof_)
      ); 

      Eigen::MatrixXd task_k_matrix {Eigen::MatrixXd::Zero(6,6)};
      task_k_matrix.diagonal() << config.task_k_0, config.task_k_1, config.task_k_2,
                                  config.task_k_3, config.task_k_4, config.task_k_5;
      is_success = compliant_controller_->setTaskKMatrix(
        task_k_matrix.block(0, 0, 6, 6)
      );

      Eigen::MatrixXd task_d_matrix {Eigen::MatrixXd::Zero(6,6)};
      task_d_matrix.diagonal() << config.task_d_0, config.task_d_1, config.task_d_2,
                                  config.task_d_3, config.task_d_4, config.task_d_5;
      is_success = compliant_controller_->setTaskDMatrix(
        task_d_matrix.block(0, 0, 6, 6)
      );

      Eigen::MatrixXd joint_k_matrix {Eigen::MatrixXd::Zero(7, 7)};
      joint_k_matrix.diagonal() << config.joint_k_0, config.joint_k_1, config.joint_k_2,
                                   config.joint_k_3, config.joint_k_4, config.joint_k_5, config.joint_k_6;
      is_success = compliant_controller_->setJointKMatrix(
        joint_k_matrix.block(0, 0, num_of_dof_, num_of_dof_)
      );

      Eigen::MatrixXd joint_d_matrix {Eigen::MatrixXd::Zero(7,7)};
      joint_d_matrix.diagonal() << config.joint_d_0, config.joint_d_1, config.joint_d_2,
                                  config.joint_d_3, config.joint_d_4, config.joint_d_5, config.joint_d_6;
      is_success = compliant_controller_->setJointDMatrix(
        joint_d_matrix.block(0, 0, num_of_dof_, num_of_dof_)
      );
      return;
    }

  }
}

#endif // COMPLIANT_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER_IMPL
