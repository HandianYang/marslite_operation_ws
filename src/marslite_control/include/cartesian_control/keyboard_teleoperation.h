#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>

#include "cartesian_control/kinematics_constants.h"
#include "utils/rpy.h"
#include "utils/tf2_listener_wrapper.h"

class KeyboardTeleoperation {
 public:
  explicit KeyboardTeleoperation(const ros::NodeHandle& nh = ros::NodeHandle());
  void run();

 private:
  void parseParameters();
  void initializePublishers();
  void setInitialGripperPose();

  void getKeyboardInput();
  void processInput();
  void processInputInPositionControl();
  void processInputInOrientationControl();
  
  inline void publishPose() {
    desired_gripper_pose_publisher_.publish(desired_gripper_pose_);
  }

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  Tf2ListenerWrapper tf2_listener_;

  // messages
  geometry_msgs::PoseStamped desired_gripper_pose_;
  std_msgs::Bool desired_gripper_status_;
  char input_key_;
  RPY desired_gripper_pose_rpy_;
  
  // flags
  bool use_sim_;  // true if running in simulation
  bool is_stopped_; // true if the program is to be terminated
  bool is_position_control_;  // true if in position control mode
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H