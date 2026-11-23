#include "compliant_controllers/model.hpp"
#include <set>
#include <stdexcept>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
namespace compliant_controllers {
ArmModel reduceToArm(const std::string& xml, const std::vector<std::string>& joints, const std::string& end_effector) {
  const std::set<std::string> keep(joints.begin(), joints.end());
  if (joints.size() != 7 || keep.size() != 7) throw std::invalid_argument("exactly seven unique arm joints required");
  pinocchio::Model full;
  pinocchio::urdf::buildModelFromXML(xml, full);
  for (const auto& name : joints)
    if (!full.existJointName(name) || full.nvs[full.getJointId(name)] != 1)
      throw std::invalid_argument("missing or non-scalar arm joint: " + name);
  std::vector<pinocchio::JointIndex> locked;
  for (pinocchio::JointIndex j = 1; j < full.joints.size(); ++j)
    if (!keep.count(full.names[j])) locked.push_back(j);
  ArmModel result;
  result.model = std::make_unique<pinocchio::Model>(pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full)));
  if (!result.model->existFrame(end_effector)) throw std::invalid_argument("missing tool frame: " + end_effector);
  for (const auto& name : joints) {
    const auto j = result.model->getJointId(name);
    result.indices.push_back(result.model->idx_vs[j]);
    result.continuous.push_back(result.model->nqs[j] == 2);
    result.lower.push_back(result.model->lowerPositionLimit[result.model->idx_qs[j]]);
    result.upper.push_back(result.model->upperPositionLimit[result.model->idx_qs[j]]);
  }
  if (result.model->nv != 7 || result.model->joints.size() != 8) throw std::invalid_argument("invalid reduced arm model");
  return result;
}
}
