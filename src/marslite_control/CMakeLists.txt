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
)

add_executable(test_joystick_teleoperation
  test/test_joystick_teleoperation.cpp
  src/cartesian_control/joystick_teleoperation.cpp
)
target_link_libraries(test_joystick_teleoperation ${catkin_LIBRARIES})

add_executable(test_keyboard_teleoperation
  test/test_keyboard_teleoperation.cpp
  src/cartesian_control/keyboard_teleoperation.cpp
)
target_link_libraries(test_keyboard_teleoperation ${catkin_LIBRARIES})

add_executable(test_detected_object_grasping
  test/test_detected_object_grasping.cpp
  src/cartesian_control/object_grasping.cpp)
target_link_libraries(test_detected_object_grasping ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################