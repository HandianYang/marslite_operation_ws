################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

project(marslite_control)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  detection_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################

catkin_package(
  CATKIN_DEPENDS
    detection_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################

add_executable(gripper_pose_publisher_node
  src/gripper_pose_publisher_node.cpp
)
target_link_libraries(gripper_pose_publisher_node
  ${catkin_LIBRARIES}
)

add_executable(motion_controller_teleoperation_node
  example/motion_controller_teleoperation_node.cpp
  src/cartesian_control/motion_controller_teleoperation.cpp
)
target_link_libraries(motion_controller_teleoperation_node
  ${catkin_LIBRARIES}
)

add_executable(keyboard_teleoperation_node
  example/keyboard_teleoperation_node.cpp
  src/cartesian_control/keyboard_teleoperation.cpp
)
target_link_libraries(keyboard_teleoperation_node
  ${catkin_LIBRARIES}
)

add_executable(shared_control_node
  example/shared_control_node.cpp
  src/shared_control/shared_control.cpp
  src/shared_control/intent_inference.cpp
  src/shared_control/controller_direction_estimator.cpp
)
target_link_libraries(shared_control_node
  ${catkin_LIBRARIES}
)