#!/usr/bin/env python3
"""Regenerate the numerical fixture from ROS1 e146030, with only a minimal ROS type/log shim.

Run inside the Jazzy validation container after sourcing ROS. The historical core is compiled
independently, including its original unwrap/utilities; none of the ported implementation is linked.
The old bare-arm fixture has exactly seven DoFs. This compares source semantics, not deployed binaries.
"""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
REV = 'e146030dc8e5bb264ccecf8efaedeb80906c10cf'
with tempfile.TemporaryDirectory(prefix='legacy_compliant_') as scratch:
    work = pathlib.Path(scratch)
    paths = ['include/compliant_controllers/joint_task_space/controller.h',
             'include/compliant_controllers/extended_joint_positions.h',
             'include/compliant_controllers/robot_state.h', 'include/compliant_controllers/utils.h',
             'src/joint_task_space/controller.cpp', 'src/extended_joint_positions.cpp', 'src/utils.cpp']
    for name in paths:
        content = subprocess.check_output(['git', '-c', f'safe.directory={ROOT}', '-C', str(ROOT), 'show', f'{REV}:{name}'], text=True)
        target = work / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    shim = work/'include/ros/ros.h'; shim.parent.mkdir()
    shim.write_text('#pragma once\n#include <iostream>\nnamespace ros { struct Duration { double value; explicit Duration(double v):value(v){} double toSec() const{return value;} }; }\n#define ROS_ERROR_THROTTLE(...) ((void)0)\n#define ROS_WARN_THROTTLE(...) ((void)0)\n')
    (work/'main.cpp').write_text(r'''
#include <fstream>
#include <iomanip>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "compliant_controllers/joint_task_space/controller.h"
#include "compliant_controllers/utils.h"
int main(int argc,char** argv){
  auto model=std::make_unique<pinocchio::Model>();pinocchio::urdf::buildModel(argv[1],*model);
  auto full=*model;pinocchio::Data data(full);
  compliant_controllers::joint_task_space::CompliantController c(std::move(model),"right_kinova_arm_tool_frame",7,true);
  std::cout<<std::setprecision(17);
  for(int tick=0;tick<300;++tick){
    const double t=tick*.001;compliant_controllers::RobotState q(7),d(7);
    q.efforts.setZero();d.efforts.setZero();q.accelerations.setZero();d.accelerations.setZero();
    for(int j=0;j<7;++j){q.positions[j]=.2+.03*std::sin(2*t+j);q.velocities[j]=.06*std::cos(2*t+j);
      d.positions[j]=q.positions[j]+.01*std::sin(3*t+j);d.velocities[j]=q.velocities[j]+.03*std::cos(3*t+j);}
    q.efforts=-pinocchio::computeGeneralizedGravity(full,data,compliant_controllers::joint_ros_to_pinocchio(q.positions,full));
    for(int j=0;j<7;++j)q.efforts[j]+=.1*std::sin(5*t+j);
    Eigen::VectorXd out=c.computeEffort(d,q,ros::Duration(.001)),ros_effort=-q.efforts;
    bool first=true;for(const auto* v:{&d.positions,&d.velocities,&q.positions,&q.velocities,&ros_effort,&out})
      for(int j=0;j<7;++j){if(!first)std::cout<<',';std::cout<<(*v)[j];first=false;}std::cout<<'\n';
  }
}
''')
    (work/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject(legacy_trace)\nset(CMAKE_CXX_STANDARD 17)\nfind_package(pinocchio REQUIRED)\nadd_executable(trace main.cpp src/joint_task_space/controller.cpp src/extended_joint_positions.cpp src/utils.cpp)\ntarget_include_directories(trace PRIVATE include)\ntarget_link_libraries(trace pinocchio::pinocchio)\n')
    subprocess.run(['cmake','-S',str(work),'-B',str(work/'build'),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(work/'build'),'-j2'],check=True)
    trace = subprocess.check_output([str(work/'build/trace'), str(ROOT/'test/urdf/oxf20_right_arm.urdf')],text=True)
    (ROOT/'test/ros2/legacy_trace.csv').write_text(f'# ROS1 source {REV}; defaults; raw torque negated into ROS convention\n'+trace)
