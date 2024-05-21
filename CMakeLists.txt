cmake_minimum_required(VERSION 3.0.2)
project(compliant_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/compliant_controllers.cpp
  src/extended_joint_positions.cpp
  src/joint_space_compliant_controller.cpp
  src/utils.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
