#ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H
#define MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "detection_msgs/DetectedObjectArray.h"
#include "shared_control/intent_inference.h"
#include "utils/tf2_listener_wrapper.h"
#include "shared_control/controller_direction_estimator.h"

class SharedControl {
public:
  explicit SharedControl(const ros::NodeHandle& nh = ros::NodeHandle());
  void run();

 private:
  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  ros::Publisher belief_visualization_publisher_;
  // ros::Publisher gripper_pose_publisher_;
  ros::Subscriber detected_objects_subscriber_;
  ros::Subscriber record_signal_subscriber_;
  ros::Subscriber user_desired_gripper_pose_subscriber_;
  ros::Subscriber user_desired_gripper_status_subscriber_;

  // self-defined class instances
  IntentInference intent_inference_;
  Tf2ListenerWrapper tf2_listener_;
  ControllerDirectionEstimator controller_direction_estimator_{10};

  // ROS messages
  geometry_msgs::TransformStamped current_gripper_transform_;
  std_msgs::Bool user_desired_gripper_status_;

  // flags
  bool begin_recording_;  // true if record_siganl is received
  bool begin_shared_control_;

 private:
  void initializePublishers();
  void initializeSubscribers();
  void publishIntentBeliefVisualization();
  void publishBlendingGripperPose();

  geometry_msgs::Point getBlendingDirection();
  geometry_msgs::Point normalizeDirection(const geometry_msgs::Point& direction);
  geometry_msgs::Point scaleDirection(const geometry_msgs::Point& direction, const double& scale);

  // Callbacks
  void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& objects);
  void recordSignalCallback(const std_msgs::Bool::ConstPtr& signal);
  void userDesiredGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& user_desired_gripper_pose);
  void userDesiredGripperStatusCallback(const std_msgs::Bool::ConstPtr& user_desired_gripper_status);
};

#endif // #ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H