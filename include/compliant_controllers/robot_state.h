/**
 * \file robot_state.h
 * \mainpage
 *   Struct that contains the robot state at a particular time stamp
 * 
 * \authors
 *   Tobit Flatscher <tobit@robots.ox.ac.uk>
 * \copyright
 *   Oxford Robotics Institute - University of Oxford (2024)
 * \license
 *   This project is released under the 3-clause BSD license.
*/

#ifndef COMPLIANT_CONTROLLERS__ROBOT_STATE
#define COMPLIANT_CONTROLLERS__ROBOT_STATE

#include <Eigen/Eigen>


namespace compliant_controllers {

  /**\class RobotState
   * \brief
   *   Holds the robot state at a particular time stamp
  */
  struct RobotState {
    public:
      /**\fn RobotState
       * \brief
       *   The state of the robot at a particular time stamp
       * 
       * \param[in] num_of_dof
       *   The degrees of freedom of all joints that the arrays should be resized to
      */
      RobotState(int const num_of_dof) {
        resize(num_of_dof);
        return;
      }
      RobotState() = default;
      RobotState(RobotState const&) = default;
      RobotState& operator= (RobotState const&) = default;
      RobotState(RobotState&&) = default;
      RobotState& operator= (RobotState&&) = default;

      /**\fn resize
       * \brief
       *   Resize the underlying arrays to a desired size
       * 
       * \param[in] num_of_dof
       *   The size that the underlying arrays should be resized to
      */
      void resize(int const num_of_dof) {
        positions.resize(num_of_dof);
        velocities.resize(num_of_dof);
        accelerations.resize(num_of_dof);
        efforts.resize(num_of_dof);
        return;
      }

      Eigen::VectorXd positions;
      Eigen::VectorXd velocities;
      Eigen::VectorXd accelerations;
      Eigen::VectorXd efforts;
  };

}

#endif // COMPLIANT_CONTROLLERS__ROBOT_STATE
