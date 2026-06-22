/**
 * \file extended_joint_positions.cpp
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

#include <Eigen/Eigen>


namespace compliant_controllers {

  ExtendedJointPositions::ExtendedJointPositions(unsigned int const number_of_dof)
  : is_initialized_{false}, number_of_dof_{number_of_dof} {
    unwrapped_joint_positions_.resize(number_of_dof_, 1);
    previous_joint_positions_.resize(number_of_dof_, 1);
    return;
  }

  bool ExtendedJointPositions::init(Eigen::VectorXd const& initial_joint_positions) {
    if (is_initialized_ == false) {
      unwrapped_joint_positions_ = normalize(initial_joint_positions);
      previous_joint_positions_ = unwrapped_joint_positions_;
      is_initialized_ = true;
      return true;
    }
    return false;
  }

  void ExtendedJointPositions::update(Eigen::VectorXd const& measured_joint_positions) {
    for (Eigen::Index i = 0; i < measured_joint_positions.size(); ++i) {
      // F4: NaN guard.  A non-finite reading is skipped entirely.  Because
      // previous_joint_positions_ is updated INSIDE the loop (below), the
      // per-joint state is left at its previous finite values and recovers
      // cleanly on the next finite reading.  Increment a counter so the
      // diagnostics topic can surface this event.  See
      // docs/diagnosis_report.md §5 (C5) and §7 (F4).
      if (!std::isfinite(measured_joint_positions(i))) {
        nan_count_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      // Continuous unwrap (std::unwrap): accumulate the wrapped delta between
      // the new reading and the previous one.  normalize() maps the delta
      // into [-pi, pi), so a 0/2pi seam crossing is read as the small
      // physical motion it really is, and the result is independent of which
      // 2pi-congruent representative the reading uses.
      double const delta {normalize(measured_joint_positions(i) - previous_joint_positions_(i))};
      unwrapped_joint_positions_(i) += delta;
      previous_joint_positions_(i) = measured_joint_positions(i);
    }
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
