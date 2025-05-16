#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

const geometry_msgs::PoseStamped kInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.712;
  pose_stamped.pose.position.y = -0.122;
  pose_stamped.pose.position.z = 1.043;
  pose_stamped.pose.orientation.x = 0.5;
  pose_stamped.pose.orientation.y = 0.5;
  pose_stamped.pose.orientation.z = 0.5;
  pose_stamped.pose.orientation.w = 0.5;
  return pose_stamped;
}();

struct RPY {
  double roll;
  double pitch;
  double yaw;
};

class CartesianControlKeyboard {
public:
  explicit CartesianControlKeyboard(const ros::NodeHandle& nh = ros::NodeHandle());
  void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher pose_pub_;
  ros::Publisher target_frame_pub_;

  geometry_msgs::PoseStamped target_pose_;
  char input_key_;
  RPY target_pose_rpy_;

  bool is_stopped_;
  bool is_position_control_;

private:
  RPY convertQuaternionMsgToRPY(const geometry_msgs::Quaternion& quaternion);
  geometry_msgs::Quaternion convertRPYToQuaternionMsg(const RPY& rpy);
  
  void getKeyboardInput();
  void processInput();
  void processInputInPositionControl();
  void processInputInOrientationControl();
  void publishPose();
};

#endif // MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_H