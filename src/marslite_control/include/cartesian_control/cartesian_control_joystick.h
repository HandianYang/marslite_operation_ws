#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
  ros::Subscriber left_joy_pose_sub_;
  ros::Subscriber left_joy_sub_;
  ros::Subscriber joint_states_sub_;

  geometry_msgs::PoseStamped previous_left_joy_pose_;
  geometry_msgs::PoseStamped current_left_joy_pose_;

  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped relative_gripper_pose_;
  geometry_msgs::PoseStamped target_gripper_pose_;

  bool is_begin_teleoperation_;
  bool is_hand_trigger_pressed_;
  bool is_index_trigger_pressed_;
  bool is_button_X_pressed_over_3s_;
  bool is_button_Y_pressed_;

  double position_scale_;
  double orientation_scale_;

private:
  // initialization
  void parseParameters();
  void initializePublishersAndSubscribers();
  void setInitialGripperPose();

  // utility functions
  RPY getRPY(const geometry_msgs::PoseStamped& pose);
  geometry_msgs::PoseStamped getScaledRelativeGripperPose();
  geometry_msgs::PoseStamped getTargetGripperPose();

  // callbacks
  void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  
};

#endif // MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H