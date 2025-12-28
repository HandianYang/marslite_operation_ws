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

add_library(cartesian_control_lib
  src/cartesian_control/motion_controller_teleoperation.cpp
  src/cartesian_control/keyboard_teleoperation.cpp
  src/utils/velocity_estimator.cpp
)
target_link_libraries(cartesian_control_lib
  ${catkin_LIBRARIES}
)

add_library(shared_control_lib
  src/shared_control/shared_control.cpp
  src/shared_control/intent_inference.cpp
)
target_link_libraries(shared_control_lib
  ${catkin_LIBRARIES}
)

add_executable(motion_controller_teleoperation_node
  example/motion_controller_teleoperation_node.cpp
)
target_link_libraries(motion_controller_teleoperation_node
  ${catkin_LIBRARIES}
  cartesian_control_lib
)

add_executable(keyboard_teleoperation_node
  example/keyboard_teleoperation_node.cpp
)
target_link_libraries(keyboard_teleoperation_node
  ${catkin_LIBRARIES}
  cartesian_control_lib
)

add_executable(shared_control_node
  example/shared_control_node.cpp
)
target_link_libraries(shared_control_node
  ${catkin_LIBRARIES}
  shared_control_lib
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################

add_executable(test_cartesian_control
  test/test_cartesian_control.cpp
)
target_link_libraries(test_cartesian_control
  ${catkin_LIBRARIES}
)

add_executable(test_first_joint_rotation
  test/test_first_joint_rotation.cpp
)
target_link_libraries(test_first_joint_rotation
  ${catkin_LIBRARIES}
  cartesian_control_lib
)

add_executable(test_detected_object_reaching
  test/test_detected_object_reaching.cpp
)
target_link_libraries(test_detected_object_reaching
  ${catkin_LIBRARIES}
)