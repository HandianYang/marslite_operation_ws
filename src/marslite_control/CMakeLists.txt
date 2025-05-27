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
  # actionlib
  # control_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  # trajectory_msgs
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
    # actionlib
    # control_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    # trajectory_msgs
  # LIBRARIES
  #   tm700_kin_lib
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cartesian_control_lib
  src/cartesian_control_keyboard.cpp
  src/cartesian_control_joystick.cpp
)

add_executable(test_cartesian_control_keyboard test/test_cartesian_control_keyboard.cpp)
target_link_libraries(test_cartesian_control_keyboard ${catkin_LIBRARIES} cartesian_control_lib)

add_executable(test_cartesian_control_joystick test/test_cartesian_control_joystick.cpp)
target_link_libraries(test_cartesian_control_joystick ${catkin_LIBRARIES} cartesian_control_lib)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################