/**
 * \file utils.h
 * \mainpage
 *   Different utility functions for the compliant controllers
 * 
 * \authors
 *   Tobit Flatscher <tobit@robots.ox.ac.uk>
 * \copyright
 *   Oxford Robotics Institute - University of Oxford (2024)
 * \license
 *   This project is released under the 3-clause BSD license.
*/

#ifndef COMPLIANT_CONTROLLERS__UTILS
#define COMPLIANT_CONTROLLERS__UTILS

#include <string>
#include <vector>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Eigen>


namespace compliant_controllers {

  /**\fn joint_ros_to_pinocchio
   * \brief
   *   Convert the joint states to the Pinocchio convention
   * 
   * \param[in] q
   *   The input joint angles
   * \param[in] model
   *   The Pinocchio robot model
   * \return
   *   The joint angles converted to the Pinocchio convention
  */
  [[nodiscard]]
  Eigen::VectorXd joint_ros_to_pinocchio(Eigen::VectorXd const& q, pinocchio::Model const& model);

}

#endif // COMPLIANT_CONTROLLERS__UTILS
