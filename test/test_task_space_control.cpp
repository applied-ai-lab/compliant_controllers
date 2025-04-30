#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "compliant_controllers/joint_task_space/controller.h"



int main(int argc, char **argv)
{

    std::string robot_description = PATH_TO_DIR + std::string("/test/urdf/oxf20_right_arm_gripper.urdf");

    auto robot_model = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(robot_description, *robot_model.get());

    int num_of_dof_ = 7;
    std::string end_effector_link = "right_kinova_arm_tool_frame";

    bool apply_gravity = true;

    Eigen::VectorXd torques = Eigen::VectorXd::Zero(num_of_dof_);

    auto compliant_controller_ = std::make_unique<compliant_controllers::joint_task_space::CompliantController>(std::move(robot_model), end_effector_link, num_of_dof_, apply_gravity);
    

    // Create the current and desired robot states
    compliant_controllers::RobotState current_state(num_of_dof_);
    compliant_controllers::RobotState desired_state(num_of_dof_);

    current_state.positions.setZero();
    current_state.velocities.setZero();
    current_state.accelerations.setZero();
    current_state.efforts.setZero();

    desired_state.positions.setZero();
    desired_state.velocities.setZero();
    desired_state.accelerations.setZero();
    desired_state.efforts.setZero();

    current_state.positions = 0.01 * Eigen::VectorXd::Ones(num_of_dof_);

    ros::Duration period(0.01);
    // Compute torque
    int no_runs = 1000;

    auto tic = std::chrono::high_resolution_clock::now();

    for (int k=0; k<no_runs; ++k)
    {
        torques = compliant_controller_->computeEffort(desired_state, current_state, period);  
    }

    auto toc = std::chrono::high_resolution_clock::now();
    double us_taken = 1e-3 * std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() / no_runs;

    std::cout << "Average computeEffort run time for " << no_runs << " runs: " <<
    us_taken << " micro seconds." << std::endl;

    std::cout << torques.transpose() << std::endl;

    return 0;
}
