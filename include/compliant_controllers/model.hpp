#pragma once
#include <memory>
#include <string>
#include <vector>
#include <pinocchio/multibody/model.hpp>
namespace compliant_controllers {
struct ArmModel {
  std::unique_ptr<pinocchio::Model> model;
  std::vector<std::size_t> indices;
  std::vector<bool> continuous;
  std::vector<double> lower, upper;
};
ArmModel reduceToArm(const std::string& urdf, const std::vector<std::string>& joints, const std::string& end_effector);
}
