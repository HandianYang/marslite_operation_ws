#ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H
#define MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "detection_msgs/DetectedObjectArray.h"
#include "shared_control/intent_inference.h"

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
  ros::Subscriber detected_objects_subscriber_;
  ros::Subscriber record_signal_subscriber_;
  ros::Subscriber user_desired_gripper_pose_subscriber_;
  ros::Subscriber user_desired_gripper_status_subscriber_;

  // ROS messages
  geometry_msgs::PoseStamped user_desired_gripper_pose_;
  geometry_msgs::PoseStamped last_user_desired_gripper_pose_;
  std_msgs::Bool user_desired_gripper_status_;
  detection_msgs::DetectedObjectArray recorded_objects_;

  IntentInference intent_inference_;

  // flags
  bool begin_recording_;  // true if record_siganl is received

private:
  void initializePublishers();
  void initializeSubscribers();
  void parseParameters();

  // Callbacks

  inline void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& msg) {
    if (begin_recording_) {
      recorded_objects_ = *msg;
      intent_inference_.setRecordedObjects(recorded_objects_);
      ROS_INFO_STREAM("Received " << recorded_objects_.objects.size() << " detected objects.");
      begin_recording_ = false;
    }
  }

  inline void recordSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
    begin_recording_ = true;
  }

  inline void userDesiredGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_user_desired_gripper_pose_ = user_desired_gripper_pose_;
    user_desired_gripper_pose_ = *msg;
    // TODO: Remove this line for shared control
    desired_gripper_pose_publisher_.publish(user_desired_gripper_pose_);
  }

  inline void userDesiredGripperStatusCallback(const std_msgs::Bool::ConstPtr& msg) {
    // Directly publish the user's desired gripper status becuase we don't 
    //   need to process it through shared control.
    user_desired_gripper_status_ = *msg;
    desired_gripper_status_publisher_.publish(user_desired_gripper_status_);
  }
};

#endif // #ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H