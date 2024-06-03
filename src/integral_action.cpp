#include "compliant_controllers/integral_action.h"

namespace compliant_controllers {

    IntegralAction::IntegralAction(int const num_controlled_dofs) : dof_{num_controlled_dofs} 
    {
        // Allocate objects
        q_error_.setZero(dof_);
        q_error_sum_.setZero(dof_);

        q_error_max_.setZero(dof_);

        action_.setZero(dof_);

        Ki_.setIdentity(dof_, dof_);
    }

    bool IntegralAction::init()
    {
        return reset();
    }

    bool IntegralAction::init(Eigen::MatrixXd const& Ki, 
                              Eigen::VectorXd const& error_max)
    {
        reset(Ki, error_max);        
        return init();
    }

    bool IntegralAction::reset()
    {
        // Zero out the sums
        q_error_sum_.setZero();
        q_error_.setZero();

        action_.setZero();
        
        return true;
    }

    bool IntegralAction::reset(Eigen::MatrixXd const& Ki, 
                               Eigen::VectorXd const& error_max)
    {
        Ki_ = Ki;
        q_error_max_ = error_max;   
        return reset();
    }

    void IntegralAction::setIntegralGain(Eigen::MatrixXd const& Ki)
    {
        Ki_ = Ki;
    }

    Eigen::VectorXd IntegralAction::advance(Eigen::VectorXd const& current_q, 
                                            Eigen::VectorXd const& desired_q) 
    {
        q_error_ = desired_q - current_q;
        q_error_sum_ += q_error_;
        // Clamp the integrated error
        clampSummedError();
        // Apply gain and return
        action_.noalias() = Ki_ * (q_error_sum_);
        return action_;
    }

    void IntegralAction::clampSummedError()
    {
        q_error_sum_ = q_error_sum_.cwiseMin(q_error_max_).cwiseMax(-q_error_max_);
    }

} //namespace compliant_controllers
