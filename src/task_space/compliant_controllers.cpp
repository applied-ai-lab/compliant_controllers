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

#include "compliant_controllers/task_space/hardware_interface_adapter.h"


// Convenient alias
using TrajectoryState = joint_trajectory_controller::JointTrajectorySegment<
    trajectory_interface::QuinticSplineSegment<double>
  >::State;
/**\class HardwareInterfaceAdapter
 * \brief
 *   This specialization is more specific than the previous specialization
 *   template <typename State> HardwareInterfaceAdapter<hardware_interface::EffortJointInterface,State>
*/
template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface,TrajectoryState>: public
      compliant_controllers::task_space::CompliantHardwareInterfaceAdapter<
        hardware_interface::EffortJointInterface,TrajectoryState
      > {
  public:
    HardwareInterfaceAdapter() = default;
    HardwareInterfaceAdapter(HardwareInterfaceAdapter const&) = default;
    HardwareInterfaceAdapter& operator= (HardwareInterfaceAdapter const&) = default;
    HardwareInterfaceAdapter(HardwareInterfaceAdapter&&) = default;
    HardwareInterfaceAdapter& operator= (HardwareInterfaceAdapter&&) = default;
};

namespace compliant_controllers {
  namespace task_space {

    /**\class TaskSpaceJointTrajectoryController
     * \brief
     *   Compliant joint trajectory controller using the compliant hardware interface adapter
    */
    class TaskSpaceJointTrajectoryController: public
      joint_trajectory_controller::JointTrajectoryController<
        trajectory_interface::QuinticSplineSegment<double>,
        hardware_interface::EffortJointInterface
      > {
      public:
        TaskSpaceJointTrajectoryController() = default;
        TaskSpaceJointTrajectoryController(TaskSpaceJointTrajectoryController const&) = default;
        TaskSpaceJointTrajectoryController& operator= (TaskSpaceJointTrajectoryController const&) = default;
        TaskSpaceJointTrajectoryController(TaskSpaceJointTrajectoryController&&) = default;
        TaskSpaceJointTrajectoryController& operator= (TaskSpaceJointTrajectoryController&&) = default;
    };
  }
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(
  compliant_controllers::task_space::TaskSpaceJointTrajectoryController, controller_interface::ControllerBase
)

