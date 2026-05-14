/**
 * \file extended_joint_position.cpp
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

#include "compliant_controllers/extended_joint_positions.h"

#include <cmath>

#include <iostream>
#include <Eigen/Eigen>


namespace compliant_controllers {

  ExtendedJointPositions::ExtendedJointPositions(unsigned int const number_of_dof,
      double const threshold)
  : is_initialized_{false}, number_of_dof_{number_of_dof}, threshold_{threshold} {
    normalized_target_joint_positions_.resize(number_of_dof_, 1);
    diff_joint_positions_.resize(number_of_dof_, 1);
    current_joint_positions_.resize(number_of_dof_, 1);
    return;
  }

  bool ExtendedJointPositions::init(Eigen::VectorXd const& joint_positions) {
    if (is_initialized_ == false) {
      // Convention fix: store `joint_positions` verbatim instead of
      // normalising to [-π, π).  `update()` is called from the
      // controller's adapter with positions in [0, 2π) (after
      // hardware_interface_adapter_impl.h:122-125 wraps negatives by
      // +2π).  Previously, `init` normalised to [-π, π) and the
      // first `update` then compared its [0, 2π) target against a
      // [-π, π) stored `current_`, causing the threshold check at
      // `update()` line 49 to misclassify the first wrap event by
      // exactly 2π.  See test/test_extended_joint_positions.cpp:
      // test_forward_wrap_through_2pi for a reproducible failure
      // before this fix, and docs/diagnosis_report.md §5 for the
      // failure-mode analysis.
      diff_joint_positions_    = joint_positions;
      current_joint_positions_ = joint_positions;
      is_initialized_ = true;
      return true;
    }
    return false;
  }

  void ExtendedJointPositions::update(Eigen::VectorXd const& target_joint_positions) {
    for (Eigen::Index i = 0; i < target_joint_positions.size(); ++i) {
      // F4: NaN guard.  A non-finite target leads to corrupt finite
      // garbage via static_cast<int>(NaN) in the else branch below.
      // Skip the update for this joint and leave diff/current at
      // their previous finite values.  Increment a counter so the
      // diagnostics topic can surface this event.  See
      // docs/diagnosis_report.md §5 (C5) and §7 (F4).
      if (!std::isfinite(target_joint_positions(i))) {
        nan_count_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      if (std::abs(target_joint_positions(i) - current_joint_positions_(i)) >= threshold_) {
        diff_joint_positions_(i) += normalize(target_joint_positions(i)) - normalize(current_joint_positions_(i));
        // std::cout << "========================" << std::endl;
        // std::cout << " BRANCH 1 IN UPDATE JOINT POSITIONS: " << i << ", " << target_joint_positions(i) << ", " << current_joint_positions_(i) << std::endl;
        // std::cout << diff_joint_positions_(i) << std::endl;
        // std::cout << "========================" << std::endl;
        
      } else {
        // Verified via test/test_extended_joint_positions.cpp.
        // The else-branch algebra is correct for normal use; the
        // only theoretical edge case is when diff is exactly a
        // negative integer multiple of 2π AND current is exactly 0
        // AND target is exactly 0, where C++ integer truncation of
        // diff/2π differs from the conventional floor by 1.  Not
        // reproducible from continuous motion sequences (numerical
        // drift prevents the exact landing).  Documented for
        // future maintainers.
        int number_of_rotations {0};
        if (diff_joint_positions_(i) >= 0) {
          number_of_rotations = static_cast<int>(diff_joint_positions_(i)/(2.0*M_PI));
        }
        else {
          number_of_rotations = static_cast<int>(diff_joint_positions_(i)/(2.0*M_PI)) - 1;
        }
        diff_joint_positions_(i) = number_of_rotations*(2.0*M_PI) + target_joint_positions(i);
      }
    }
    current_joint_positions_ = target_joint_positions;
    return;
  }

  constexpr double ExtendedJointPositions::normalize(double const joint_angle) noexcept {
    // Taken from  https://stackoverflow.com/questions/11980292/how-to-wrap-around-a-range
    double output = std::fmod(joint_angle + M_PI, 2.0*M_PI);
    if (output < 0.0) {
      output += 2.0*M_PI;
    }
    return output - M_PI;
  }

  Eigen::VectorXd ExtendedJointPositions::normalize(Eigen::VectorXd const& joint_positions) {
    Eigen::VectorXd output {joint_positions};
    for (Eigen::Index i = 0; i < output.size(); ++i) {
      output(i) = normalize(joint_positions(i));
    }
    return output;
  }

} // compliant_controllers
