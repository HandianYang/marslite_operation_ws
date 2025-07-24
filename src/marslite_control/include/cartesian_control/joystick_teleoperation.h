#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "cartesian_control/kinematics_constants.h"
#include "utils/rpy.h"
#include "utils/tf2_listener_wrapper.h"

class JoystickTeleoperationWrapper {
public:
  explicit JoystickTeleoperationWrapper(const ros::NodeHandle& nh = ros::NodeHandle());

  void teleoperate();
  void calculateDesiredGripperDisplacement();
  void calculateDesiredGripperPose();

private:
  const double kTriggerThreshold = 0.95;

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_displacement_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  ros::Subscriber left_joy_pose_subscriber_;
  ros::Subscriber left_joy_subscriber_;
  Tf2ListenerWrapper tf2_listener_;

  // ROS messages
  geometry_msgs::TransformStamped base_link_to_tm_gripper_transform_;
  geometry_msgs::PoseStamped previous_left_joy_pose_;
  geometry_msgs::PoseStamped current_left_joy_pose_;
  geometry_msgs::PoseStamped current_gripper_pose_;
  geometry_msgs::PoseStamped desired_gripper_displacement_;
  geometry_msgs::PoseStamped desired_gripper_pose_;
  std_msgs::Bool desired_gripper_status_;

  // flags
  bool is_position_change_enabled_;
  bool is_orientation_change_enabled_;
  bool use_shared_controller_;  // false if using pure teleoperation

  // parameters
  double position_scale_;
  double orientation_scale_;

private:
  // initialization
  void parseParameters();
  void initializePublishers();
  void initializeSubscribers();

  inline void publishDesiredGripperDIsplacement() {
    desired_gripper_displacement_publisher_.publish(desired_gripper_displacement_);
  }

  inline void publishDesiredGripperPose() {
    desired_gripper_pose_publisher_.publish(desired_gripper_pose_);
  }

  // utility operations (supports calculateDesiredGripperDisplacement())
  geometry_msgs::Point getPositionDifference() const;
  geometry_msgs::Point scalePositionDifference(const geometry_msgs::Point& position_difference) const;
  RPY getRPYDifference() const;
  RPY getRPYFromPose(const geometry_msgs::PoseStamped& pose) const;
  double restrictAngleWithinPI(const double& angle) const;
  RPY scaleAndTransformRPYDifference(const RPY& rpy_difference) const;
  geometry_msgs::Quaternion convertRPYToQuaternion(const RPY& rpy) const;

  //
  geometry_msgs::Point applyPositionDisplacement() const;
  geometry_msgs::Quaternion applyOrientationDisplacement() const;

  // callbacks
  void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_TELEOPERATION_H