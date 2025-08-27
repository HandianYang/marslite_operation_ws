#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "cartesian_control/kinematics_constants.h"
#include "utils/rpy.h"
#include "utils/tf2_listener_wrapper.h"

class MotionControllerTeleoperation {
public:
  explicit MotionControllerTeleoperation(const ros::NodeHandle& nh = ros::NodeHandle());

  void teleoperate();
  void calculateDesiredGripperPose();

private:
  const double kTriggerThreshold = 0.95;

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  ros::Publisher record_signal_publisher_;
  ros::Publisher mobile_platform_velocity_publisher_;
  ros::Subscriber left_controller_pose_subscriber_;
  ros::Subscriber left_controller_joy_subscriber_;
  Tf2ListenerWrapper tf2_listener_;

  // ROS messages
  geometry_msgs::TransformStamped current_gripper_transform_;
  geometry_msgs::PoseStamped initial_left_controller_pose_;
  geometry_msgs::PoseStamped current_left_controller_pose_;
  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped desired_gripper_pose_;
  geometry_msgs::Twist mobile_platform_velocity_;
  std_msgs::Bool desired_gripper_status_;
  std_msgs::Bool record_signal_;

  // flags
  bool is_begin_teleoperation_;  // true if teleoperation has not started yet
  bool is_position_change_enabled_;
  bool is_orientation_change_enabled_;

  // parameters
  double position_scale_;
  double orientation_scale_;
  double linear_velocity_scale_;
  double angular_velocity_scale_;
  bool use_shared_controller_;  // false if using pure teleoperation

private:
  // initialization
  void parseParameters();
  void initializePublishers();
  void initializeSubscribers();
  void setInitialGripperPose();

  // utility operations (supports teleoperateOnce())
  
  inline void lookupCurrentGripperTransform() {
    current_gripper_transform_ = tf2_listener_.lookupTransform("base_link", "tm_gripper");
  }

  inline void resetPositionalMovement() {
    desired_gripper_pose_.pose.position.x = current_gripper_transform_.transform.translation.x;
    desired_gripper_pose_.pose.position.y = current_gripper_transform_.transform.translation.y;
    desired_gripper_pose_.pose.position.z = current_gripper_transform_.transform.translation.z;
  }

  inline void resetOrientationalMovement() {
    desired_gripper_pose_.pose.orientation = current_gripper_transform_.transform.rotation;
  }

  inline void publishDesiredGripperPose() {
    desired_gripper_pose_publisher_.publish(desired_gripper_pose_);
  }

  inline void publishMobilePlatformVelocity() {
    mobile_platform_velocity_publisher_.publish(mobile_platform_velocity_);
  }

  // utility operations (supports calculateDesiredGripperPose())
  geometry_msgs::Vector3 getPositionDifference();
  RPY getOrientationDifference();
  RPY getRPYFromPose(const geometry_msgs::PoseStamped& pose);
  double restrictAngleWithinPI(const double& angle);

  // callbacks
  void leftControllerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftControllerJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void toggleGripperStatus();
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H