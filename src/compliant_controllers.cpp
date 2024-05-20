/**
 * \file compliant_controllers.cpp
 * \mainpage
 *   Instantiation of the compliant joint-trajectory controllers
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

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/fwd.hpp>

#include <hardware_interface/joint_command_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include "compliant_controllers/hardware_interface_adapter.h"


namespace compliant_controllers {

  /**\class JointTrajectoryController
   * \brief
   *   Compliant joint trajectory controller using the compliant hardware interface adapter
  */
  class JointTrajectoryController: public
    joint_trajectory_controller::JointTrajectoryController<
      trajectory_interface::QuinticSplineSegment<double>,
      hardware_interface::EffortJointInterface,
      compliant_controllers::CompliantHardwareInterfaceAdapter
    > {
    public:
      JointTrajectoryController() = default;
      JointTrajectoryController(JointTrajectoryController const&) = default;
      JointTrajectoryController& operator= (JointTrajectoryController const&) = default;
      JointTrajectoryController(JointTrajectoryController&&) = default;
      JointTrajectoryController& operator= (JointTrajectoryController&&) = default;
  };

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(
  compliant_controllers::JointTrajectoryController, controller_interface::ControllerBase
)

