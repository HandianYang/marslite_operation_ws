#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "cartesian_control/kinematics_constants.h"
#include "utils/rpy.h"

class KeyboardTeleoperationWrapper {
public:
  explicit KeyboardTeleoperationWrapper(const ros::NodeHandle& nh = ros::NodeHandle());
  void teleoperate();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher target_frame_publisher_;

  geometry_msgs::PoseStamped target_pose_;
  char input_key_;
  RPY target_pose_rpy_;

  bool is_stopped_;
  bool is_position_control_;

private:
  // main operations (supports teleoperate())
  void getKeyboardInput();
  void processInput();
  void publishPose();

  // sub-operations (supports processInput())
  void processInputInPositionControl();
  void processInputInOrientationControl();
  RPY convertQuaternionMsgToRPY(const geometry_msgs::Quaternion& quaternion);
  geometry_msgs::Quaternion convertRPYToQuaternionMsg(const RPY& rpy);
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_TELEOPERATION_H