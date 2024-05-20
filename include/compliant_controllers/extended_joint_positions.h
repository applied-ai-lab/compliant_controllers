/**
 * \file extended_joint_positions.h
 * \mainpage
 *   Helpers for joint position calculations
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

#ifndef COMPLIANT_CONTROLLERS__EXTENDED_JOINT_POSITIONS
#define COMPLIANT_CONTROLLERS__EXTENDED_JOINT_POSITIONS

#include <Eigen/Eigen>


namespace compliant_controllers {

  /**\class ExtendedJointPositions
   * \brief
   *   Helper class for estimating difference joint positions
  */
  class ExtendedJointPositions {
    public:
      /**\fn ExtendedJointPositions
       * \brief
       *   Constructor that allocates required memory
       *
       * \param[in] number_of_dof
       *   Degrees of freedom of all of the joints combined
       * \param[in] threshold
       *   Threshold for the joint angles
      */
      ExtendedJointPositions(unsigned int const number_of_dof, double const threshold = 3*M_PI/2);
      ExtendedJointPositions() = delete;
      ExtendedJointPositions(ExtendedJointPositions const&) = default;
      ExtendedJointPositions& operator= (ExtendedJointPositions const&) = default;
      ExtendedJointPositions(ExtendedJointPositions&&) = default;
      ExtendedJointPositions& operator= (ExtendedJointPositions&&) = default;

      /**\fn init
       * \brief
       *   Initialize the class with new joint angles
       * \warning
       *   Only works if called for the first time, else the value is discarded
       *
       * \param[in] joint_positions
       *   Joint positions that the class should be initialized with
       * \return
       *   Boolean value indicating success (true) or failure (false)
      */
      [[nodiscard]]
      bool init(Eigen::VectorXd const& joint_positions);

      /**\fn update
       * \brief
       *   Compute the new joint positions. For getting the estimated positions call the getter function.
       *
       * \param[in] target_joint_positions
       *   The desired target 
       * \return
       *   Boolean value indicating success (true) or failure (false)
      */
      void update(Eigen::VectorXd const& target_joint_positions);

      /**\fn getPositions
       * \brief
       *   Getter for the newly computed joint positions
       *
       * \return
       *   The newly computed joint positions
      */
      [[nodiscard]]
      Eigen::VectorXd getPositions() const noexcept {
        return diff_joint_positions_;
      }

      /**\fn isInitialized
       * \brief
       *   Check whether this structure was already initialized successfully
       *
       * \return
       *   Boolean value indicating initialization (true) or not (false)
      */
      [[nodiscard]]
      bool isInitialized() const noexcept {
        return is_initialized_;
      }

    protected:
      /**\fn normalize
       * \brief
       *   Normalize a single joint angle to the domain [-pi, pi)
       *
       * \param[in] joint_angle
       *   The single joint angle to be normalized
       * \return
       *   The normalized joint angle [-pi, pi)
      */
      [[nodiscard]]
      static constexpr double normalize(double const joint_angle) noexcept;

      /**\fn normalize
       * \brief
       *   Normalize a vector of joint angles to the domain [-pi, pi)
       *
       * \param[in] joint_angles
       *   The joint angles to be normalized
       * \return
       *   The normalized joint angles [-pi, pi)
      */
      [[nodiscard]]
      static Eigen::VectorXd normalize(Eigen::VectorXd const& joint_angles);

      bool is_initialized_;
      unsigned int number_of_dof_;
      double threshold_;

      Eigen::VectorXd normalized_target_joint_positions_;
      Eigen::VectorXd diff_joint_positions_;
      Eigen::VectorXd current_joint_positions_;
  };

}

#endif // COMPLIANT_CONTROLLERS__EXTENDED_JOINT_POSITIONS
