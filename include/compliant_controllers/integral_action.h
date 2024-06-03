/**
 * \file integral_action.h
 * \mainpage
 *   Compliant controller in joint space
 * 
 * \authors
 *   Tobit Flatscher <tobit@robots.ox.ac.uk>
 *   Alexander Mitchell <mitch@robots.ox.ac.uk>
 *   Rishabh Madan <rm773@cornell.edu>
 *   Rajat Kumar Jenamani <rj277@cornell.edu>
 * \copyright
 *   Oxford Robotics Institute - University of Oxford (2024)
 *   EmPRISE Lab - Cornell University (2023)
 * \license
 *   This project is released under the 3-clause BSD license.
*/

#ifndef COMPLIANT_CONTROLLERS__INTEGRAL_ACTION
#define COMPLIANT_CONTROLLERS__INTEGRAL_ACTION
#pragma once

#include <Eigen/Eigen>

namespace compliant_controllers {

    class IntegralAction {
        public:
            IntegralAction(int const num_controlled_dofs);

            // Functional methods
            bool init();
            bool init(Eigen::MatrixXd const& Ki, 
                      Eigen::VectorXd const& error_max);

            Eigen::VectorXd advance(Eigen::VectorXd const& current_q, 
                                    Eigen::VectorXd const& desired_q);

            bool reset();
            bool reset(Eigen::MatrixXd const& Ki, 
                       Eigen::VectorXd const& error_max);

            // Helper methods
            void clampSummedError();

            void setIntegralGain(Eigen::MatrixXd const& Ki);

        
        private:
            // Degrees of freedom
            int dof_;

            // Integral Gain
            Eigen::MatrixXd Ki_;

            // Joint states and erros
            Eigen::VectorXd q_error_;
            Eigen::VectorXd q_error_sum_;
            // Integral action
            Eigen::VectorXd action_;
            // Max joint state error
            Eigen::VectorXd q_error_max_;

    };
    
} // namespace name

#endif //COMPLIANT_CONTROLLERS__INTEGRAL_ACTION