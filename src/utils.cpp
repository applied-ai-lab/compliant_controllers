/**
 * \file utils.cpp
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

#include "compliant_controllers/utils.h"

#include <cmath>
#include <stdexcept>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Eigen>


namespace compliant_controllers {

  Eigen::VectorXd joint_ros_to_pinocchio(Eigen::VectorXd const& q, pinocchio::Model const& model) {
    Eigen::VectorXd q_pin {model.nq};
    joint_ros_to_pinocchio(q, model, q_pin);
    return q_pin;
  }

  void joint_ros_to_pinocchio(Eigen::VectorXd const& q, pinocchio::Model const& model, Eigen::VectorXd& q_pin) {
    if (q.size() != model.nv || q_pin.size() != model.nq) throw std::invalid_argument("Pinocchio configuration size");
    for (std::size_t i = 0; i < model.joints.size() - 1; i++) {
      pinocchio::JointIndex const jidx {model.getJointId(model.names[i + 1])};
      int const qidx {model.idx_qs[jidx]};
      // 2 corresponds to continuous joints in Pinocchio
      if (model.nqs[jidx] == 2) {
        q_pin(qidx) = std::cos(q(i));
        q_pin(qidx + 1) = std::sin(q(i));
      } else {
        q_pin(qidx) = q(i);
      }
    }
  }

} // compliant_controllers
