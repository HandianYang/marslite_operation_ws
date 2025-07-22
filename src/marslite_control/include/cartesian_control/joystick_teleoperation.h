#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
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

  inline void teleoperate() {
    while (nh_.ok()) {
      this->teleoperateOnce();
      rate_.sleep();
      ros::spinOnce();
    }
  }
  void teleoperateOnce();
  void calculateTargetGripperPose();

private:
  const double kTriggerThreshold = 0.95;

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  ros::Subscriber left_joy_pose_subscriber_;
  ros::Subscriber left_joy_subscriber_;
  Tf2ListenerWrapper tf2_listener_;

  // ROS messages
  geometry_msgs::TransformStamped base_link_to_tm_gripper_transform_;
  geometry_msgs::PoseStamped initial_left_joy_pose_;
  geometry_msgs::PoseStamped current_left_joy_pose_;
  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped desired_gripper_pose_;
  std_msgs::Bool desired_gripper_status_;

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
  void initializePublishers();
  void initializeSubscribers();
  void setInitialGripperPose();

  // utility operations (supports teleoperateOnce())
  void stopGripperPositionalMovement();
  void stopGripperOrientationalMovement();
  void publishDesiredGripperPose(const geometry_msgs::PoseStamped& desired_gripper_pose);

  // utility operations (supports calculateTargetGripperPose())
  geometry_msgs::Vector3 getPositionDifference();
  RPY getOrientationDifference();
  RPY getRPYFromPose(const geometry_msgs::PoseStamped& pose);
  double restrictAngleWithinPI(const double& angle);

  // callbacks
  void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_TELEOPERATION_H