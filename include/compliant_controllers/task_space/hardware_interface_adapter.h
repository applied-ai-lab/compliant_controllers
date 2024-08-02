/**
 * \file hardware_interface_adapter.h
 * \mainpage
 *   Contains a custom hardware interface adapter for the compliant controller
 *   This allows us to override the behavior of the default controller
 * 
 * \authors
 *   Tobit Flatscher <tobit@robots.ox.ac.uk>
 * \copyright
 *   Oxford Robotics Institute - University of Oxford (2024)
 * \license
 *   This project is released under the 3-clause BSD license.
*/

#ifndef COMPLIANT_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER
#define COMPLIANT_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER
#pragma once

#include <memory>
#include <vector>

// Pinocchio has to be included before ROS
// See https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/fwd.hpp>

#include <dynamic_reconfigure/server.h>
#include <Eigen/Eigen>
#include <hardware_interface/joint_command_interface.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <ros/ros.h>

#include "compliant_controllers/TaskSpaceCompliantControllerConfig.h"
#include "compliant_controllers/task_space/task_space_compliant_controller.h"
#include "compliant_controllers/robot_state.h"


namespace compliant_controllers {
  namespace task_space {

    // This is needed as we only want to specialize the effort implementation
    template <typename HardwareInterface, typename State>
    class CompliantHardwareInterfaceAdapter;

    /**\class CompliantHardwareInterfaceAdapter
     * \brief
     *   The hardware interface adapter that maps the position/velocity set-point to an effort command
     *   by using the compliant controller
     * 
     * \tparam State
     *   The state representation used for the state of the controller (e.g. velocity, acceleration)
    */
    template <typename State>
    class CompliantHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State> {
      public:
        /**\fn CompliantHardwareInterfaceAdapter
         * \brief
         *   Constructor that allocates all necessary structures
        */
        CompliantHardwareInterfaceAdapter();
        CompliantHardwareInterfaceAdapter(CompliantHardwareInterfaceAdapter const&) = default;
        CompliantHardwareInterfaceAdapter& operator= (CompliantHardwareInterfaceAdapter const&) = default;
        CompliantHardwareInterfaceAdapter(CompliantHardwareInterfaceAdapter&&) = default;
        CompliantHardwareInterfaceAdapter& operator= (CompliantHardwareInterfaceAdapter&&) = default;

        /**\fn init
         * \brief
         *   Constructor that allocates required memory
         *
         * \param[in] number_of_dof
         *   Degrees of freedom of all of the joints combined
         * \param[in] threshold
         *   Threshold for the joint angles
        */
        [[nodiscard]]
        bool init(std::vector<hardware_interface::JointHandle>& joint_handles,
                  ros::NodeHandle& controller_nh);

        /**\fn starting
         * \brief
         *   Start the controller
         *
         * \param[in] time
         *   The time when the controller was started
        */
        void starting(ros::Time const& /*time*/);

        /**\fn stopping
         * \brief
         *   Stop the controller
         *
         * \param[in] time
         *   The time when the controller was stopped
        */
        void stopping(ros::Time const& /*time*/);

        /**\fn updateCommand
         * \brief
         *   Update the command value by writing to the internally stored joint handles
         *
         * \param[in] time
         *   The time that the update command was called
         * \param[in] period
         *   The period of the controller
         * \param[in] desired_state
         *   The desired state that the controller should be set to
         * \param[in] state_error
         *   The current state error
        */
        void updateCommand(ros::Time const& /*time*/, ros::Duration const& period,
                          State const& desired_state, State const& /*state_error*/);

      protected:
        /**\fn dynamicReconfigureCallback
         * \brief
         *   Dynamically reconfigure the joint stiffness and other controller parameters
         *
         * \param[in] config
         *   The config to be parsed
         * \param[in] period
         *   The period of the controller
         * \param[in] desired_state
         *   The desired state that the controller should be set to
         * \param[in] state_error
         *   The current state error
        */
        void dynamicReconfigureCallback(TaskSpaceCompliantControllerConfig const& config,
          uint32_t const level);

        std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
        std::unique_ptr<TaskSpaceCompliantController> compliant_controller_;
        bool execute_default_command_;
        std::size_t num_of_dof_;
        RobotState desired_state_;
        RobotState current_state_;
        Eigen::VectorXd command_effort_;
        dynamic_reconfigure::Server<
          TaskSpaceCompliantControllerConfig> dynamic_reconfigure_server_;
        dynamic_reconfigure::Server<
          TaskSpaceCompliantControllerConfig>::CallbackType dynamic_reconfigure_callback_;
    };
  }
}

#endif // COMPLIANT_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER

#include "compliant_controllers/task_space/hardware_interface_adapter_impl.h"
