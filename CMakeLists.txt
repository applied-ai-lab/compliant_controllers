cmake_minimum_required(VERSION 3.0.2)
project(compliant_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(BUILD_TESTS ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  driver_base
  dynamic_reconfigure
  hardware_interface
  joint_trajectory_controller
  pluginlib
  roscpp
)
find_package(pinocchio REQUIRED)

generate_dynamic_reconfigure_options(
  config/JointSpaceParams.cfg
  config/JointTaskSpaceParams.cfg
  config/TaskSpaceParams.cfg
)

catkin_package( 
  CATKIN_DEPENDS
  controller_interface
  driver_base
  dynamic_reconfigure
  hardware_interface
  joint_trajectory_controller
  pluginlib
  roscpp
)
find_package(Eigen3 REQUIRED)

catkin_install_python(PROGRAMS
    scripts/switch_mode.py
    scripts/test_controller_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Compiled as three separate libraries to avoid linker error with joint_trajectory_controller internals
add_library(joint_space_compliant_controller
  src/joint_space/compliant_controllers.cpp
  src/joint_space/controller.cpp
  src/extended_joint_positions.cpp
  src/utils.cpp
)
target_compile_definitions(joint_space_compliant_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_link_libraries(joint_space_compliant_controller
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)
add_dependencies(joint_space_compliant_controller ${PROJECT_NAME}_gencfg)

add_library(joint_task_space_compliant_controller
  src/joint_task_space/compliant_controllers.cpp
  src/joint_task_space/controller.cpp
  src/extended_joint_positions.cpp
  src/utils.cpp
)
target_compile_definitions(joint_task_space_compliant_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_link_libraries(joint_task_space_compliant_controller
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)
add_dependencies(joint_task_space_compliant_controller ${PROJECT_NAME}_gencfg)

add_library(task_space_compliant_controller
  src/task_space/compliant_controllers.cpp
  src/task_space/task_space_compliant_controller.cpp
  src/extended_joint_positions.cpp
  src/utils.cpp
)
target_compile_definitions(task_space_compliant_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_link_libraries(task_space_compliant_controller
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)
add_dependencies(task_space_compliant_controller ${PROJECT_NAME}_gencfg)

install(TARGETS joint_space_compliant_controller joint_task_space_compliant_controller task_space_compliant_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(BUILD_TESTS)

  add_library(joint_task_space_compliant_controller_test
    src/joint_task_space/controller.cpp
    src/extended_joint_positions.cpp
    src/utils.cpp
  )

  add_executable(${PROJECT_NAME}_joint_taskspace_ctrl_test 
                    test/test_task_space_control.cpp)
  
  target_link_libraries(${PROJECT_NAME}_joint_taskspace_ctrl_test 
                          joint_task_space_compliant_controller_test
                          ${catkin_LIBRARIES}
                          pinocchio::pinocchio)

  target_compile_definitions(${PROJECT_NAME}_joint_taskspace_ctrl_test PUBLIC PATH_TO_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

endif()

