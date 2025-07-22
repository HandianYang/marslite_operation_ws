#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>

#include "cartesian_control/kinematics_constants.h"
#include "utils/rpy.h"

class KeyboardTeleoperationWrapper {
public:
  explicit KeyboardTeleoperationWrapper(const ros::NodeHandle& nh = ros::NodeHandle());
  void teleoperate();

private:
  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;

  // messages
  geometry_msgs::PoseStamped desired_gripper_pose_;
  std_msgs::Bool desired_gripper_status_;
  char input_key_;
  RPY desired_gripper_pose_rpy_;
  
  // flags
  bool is_stopped_;
  bool is_position_control_;

private:
  // initialization
  void initializePublishers();
  void setInitialGripperPose();

  // main operations (supports teleoperate())
  void getKeyboardInput();
  void processInput();
  void publishPose();

  // utility operations (supports processInput())
  void processInputInPositionControl();
  void processInputInOrientationControl();
  RPY convertQuaternionMsgToRPY(const geometry_msgs::Quaternion& quaternion);
  geometry_msgs::Quaternion convertRPYToQuaternionMsg(const RPY& rpy);
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H