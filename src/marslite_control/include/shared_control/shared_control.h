#ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H
#define MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "detection_msgs/DetectedObjectArray.h"
#include "shared_control/intent_inference.h"
#include "utils/velocity_estimator.h"
#include "utils/tf2_listener_wrapper.h"
#include "utils/rpy.h"

class SharedControl {
 public:
  explicit SharedControl(const ros::NodeHandle& nh = ros::NodeHandle());
  void run();

 private:
  void initializePublishers();
  void initializeSubscribers();
  void publishIntentBeliefVisualization();
  void publishBlendingGripperPose();
  geometry_msgs::PoseStamped getTargetPose();
  geometry_msgs::Point getTargetPosition();
  geometry_msgs::Quaternion getTargetOrientation();

  void currentGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& objects);
  void recordSignalCallback(const std_msgs::Bool::ConstPtr& signal);
  void positionSafetyButtonSignalCallback(const std_msgs::Bool::ConstPtr& signal);
  void orientationSafetyButtonSignalCallback(const std_msgs::Bool::ConstPtr& signal);
  void userDesiredGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& user_desired_gripper_pose);
  void userDesiredGripperStatusCallback(const std_msgs::Bool::ConstPtr& user_desired_gripper_status);
  void userCommandVelocityCallback(const geometry_msgs::Vector3::ConstPtr& user_command_velocity);

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  ros::Publisher belief_visualization_publisher_;
  ros::Publisher reset_pose_signal_publisher_;
  ros::Publisher gripper_motion_state_publisher_;
  ros::Subscriber current_gripper_pose_subscriber_;
  ros::Subscriber detected_objects_subscriber_;
  ros::Subscriber record_signal_subscriber_;
  ros::Subscriber position_safety_button_signal_subscriber_;
  ros::Subscriber orientation_safety_button_signal_subscriber_;
  ros::Subscriber user_desired_gripper_pose_subscriber_;
  ros::Subscriber user_desired_gripper_status_subscriber_;
  ros::Subscriber user_command_velocity_subscriber_;

  // self-defined class instances
  IntentInference intent_inference_;

  // ROS messages
  geometry_msgs::PoseStamped current_gripper_pose_;
  geometry_msgs::PoseStamped user_desired_gripper_pose_;
  std_msgs::Bool user_desired_gripper_status_;
  std_msgs::Bool position_safety_button_signal_;
  std_msgs::Bool orientation_safety_button_signal_;

  // flags
  bool begin_recording_;  // true if record_siganl is received
  bool is_previously_locked_;  // true if intent_inference_ is in LOCK state
};

#endif // #ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H