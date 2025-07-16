#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

const static double kTriggerThreshold = 0.95;

struct RPY {
  double roll;
  double pitch;
  double yaw;
};

class CartesianControlJoystick {
public:
  explicit CartesianControlJoystick(const ros::NodeHandle& nh = ros::NodeHandle());
  void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher target_frame_pub_;
  ros::Publisher gripper_pub_;
  ros::Subscriber left_joy_pose_sub_;
  ros::Subscriber left_joy_sub_;
  ros::Subscriber joint_states_sub_;

  geometry_msgs::PoseStamped initial_left_joy_pose_;
  geometry_msgs::PoseStamped current_left_joy_pose_;
  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped relative_gripper_pose_;
  geometry_msgs::PoseStamped target_gripper_pose_;
  std_msgs::Bool gripper_;

  // flags
  bool is_begin_teleoperation_;
  bool is_position_change_enabled_;
  bool is_orientation_change_enabled_;

  // parameters
  double position_scale_;
  double orientation_scale_;

private:
  // initialization
  void parseParameters();
  void initializePublishersAndSubscribers();
  void setInitialGripperPose();

  // utility functions
  RPY getRPYFromPose(const geometry_msgs::PoseStamped& pose);
  double restrictAngleWithinPI(const double& angle);

  // main operations
  geometry_msgs::Vector3 getPositionDifference();
  RPY getOrientationDifference();
  void calculateTargetGripperPose();

  // callbacks
  void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif // MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H