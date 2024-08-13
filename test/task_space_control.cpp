#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "compliant_controllers/task_space/task_space_compliant_controller.h"


namespace compliant_controllers {
    namespace task_space {

        int main(int argc, char **argv)
        {

            std::string robot_description = PATH_TO_DIR + std::string("test/urdf/gen3_robot.urdf");

            auto robot_model = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModel(robot_description, *robot_model.get());

            int num_of_dof_ = 7;
            std::string end_effector_link = "tool_frame";

            auto compliant_controller_ = std::make_unique<TaskSpaceCompliantController>(std::move(robot_model), end_effector_link, num_of_dof_);
            
            

            return 0;
        }

    }
}