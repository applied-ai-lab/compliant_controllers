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

#include <atomic>
#include <cstdint>

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
      */
      ExtendedJointPositions(unsigned int const number_of_dof);
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
       * \param[in] initial_joint_positions
       *   Joint positions that the class should be initialized with
       * \return
       *   Boolean value indicating success (true) or failure (false)
      */
      [[nodiscard]]
      bool init(Eigen::VectorXd const& initial_joint_positions);

      /**\fn update
       * \brief
       *   Update the continuous (unwrapped) joint positions from the latest
       *   measured reading. For getting the estimated positions call the getter function.
       *
       * \param[in] measured_joint_positions
       *   The latest measured joint positions (wrapped sensor readings)
      */
      void update(Eigen::VectorXd const& measured_joint_positions);

      /**\fn getPositions
       * \brief
       *   Getter for the newly computed joint positions
       *
       * \return
       *   The newly computed joint positions
      */
      [[nodiscard]]
      Eigen::VectorXd getPositions() const noexcept {
        return unwrapped_joint_positions_;
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

      /**\fn getNanCount
       * \brief
       *   Diagnostic accessor — number of times update() received a
       *   non-finite target and skipped the per-joint update to avoid
       *   propagating NaN into unwrapped_joint_positions_.  See F4 in
       *   docs/diagnosis_report.md.
       *
       * \return
       *   Cumulative count of non-finite targets received since
       *   construction.
      */
      [[nodiscard]]
      std::uint64_t getNanCount() const noexcept {
        return nan_count_.load(std::memory_order_relaxed);
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

      Eigen::VectorXd unwrapped_joint_positions_;
      Eigen::VectorXd previous_joint_positions_;

      // Diagnostic counter for non-finite target inputs.  See F4 in
      // docs/diagnosis_report.md.  Atomic so the read in
      // getNanCount() (called from the adapter's diagnostics timer
      // on the spinner thread) is safe vs the increment on the
      // controller-manager update thread.
      std::atomic<std::uint64_t> nan_count_{0};
  };

}

#endif // COMPLIANT_CONTROLLERS__EXTENDED_JOINT_POSITIONS
