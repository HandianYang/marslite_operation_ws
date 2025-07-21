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

  inline void teleoperate()
  {
    while (nh_.ok()) {
      this->teleoperateOnce();
      rate_.sleep();
      ros::spinOnce();
    }
  }
  void teleoperateOnce();
  void calculateTargetGripperPose();
  
  inline void setGripperPositionToCurrent()
  {
    target_gripper_pose_.pose.position.x = base_link_to_tm_gripper_transform_.transform.translation.x;
    target_gripper_pose_.pose.position.y = base_link_to_tm_gripper_transform_.transform.translation.y;
    target_gripper_pose_.pose.position.z = base_link_to_tm_gripper_transform_.transform.translation.z;
  }

  inline void setGripperOrientationToCurrent()
  {
    target_gripper_pose_.pose.orientation = base_link_to_tm_gripper_transform_.transform.rotation;
  }

  inline void publishTargetGripperPose(
      const geometry_msgs::PoseStamped& target_gripper_pose = kInitialGripperPose)
  {
    target_frame_publisher_.publish(target_gripper_pose);
  }

  inline geometry_msgs::PoseStamped getTargetGripperPose() const
  {
    return target_gripper_pose_;
  }

private:
  const double kTriggerThreshold = 0.95;

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher target_frame_publisher_;
  ros::Publisher gripper_publisher_;
  ros::Subscriber left_joy_pose_subscriber_;
  ros::Subscriber left_joy_subscriber_;
  Tf2ListenerWrapper tf2_listener_;

  // ROS messages
  geometry_msgs::PoseStamped initial_left_joy_pose_;
  geometry_msgs::PoseStamped current_left_joy_pose_;
  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped target_gripper_pose_;
  geometry_msgs::TransformStamped base_link_to_tm_gripper_transform_;
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

  // main operations (supports calculateTargetGripperPose())
  geometry_msgs::Vector3 getPositionDifference();
  RPY getOrientationDifference();
  RPY getRPYFromPose(const geometry_msgs::PoseStamped& pose);
  double restrictAngleWithinPI(const double& angle);

  // callbacks
  void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_TELEOPERATION_H