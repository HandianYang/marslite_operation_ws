#include "shared_control/shared_control.h"

#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

SharedControl::SharedControl(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(10)), begin_recording_(false), 
      intent_inference_(0.1) {
  this->initializePublishers();
  this->initializeSubscribers();
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void SharedControl::run() {
  while (nh_.ok()) {
    intent_inference_.updateBelief();
    this->publishIntentBeliefVisualization();
    this->publishBlendingGripperPose();
    
    rate_.sleep();
    ros::spinOnce();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void SharedControl::initializePublishers() {
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/cartesian_control/target_frame", 1
  );
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/gripper/cmd_gripper", 1
  );
  belief_visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/marslite_control/intent_belief_marker", 1
  );
  reset_pose_signal_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/marslite_control/lock_state_signal", 1
  );
}

void SharedControl::initializeSubscribers() {
  current_gripper_pose_subscriber_ = nh_.subscribe(
      "/marslite_control/current_gripper_pose", 1,
      &SharedControl::currentGripperPoseCallback, this
  );
  detected_objects_subscriber_ = nh_.subscribe(
      "/yolo/detected_objects", 1,
      &SharedControl::detectedObjectsCallback, this
  );
  record_signal_subscriber_ = nh_.subscribe(
      "/marslite_control/record_signal", 1,
      &SharedControl::recordSignalCallback, this
  );
  user_desired_gripper_pose_subscriber_ = nh_.subscribe(
      "/marslite_control/user_desired_gripper_pose", 1,
      &SharedControl::userDesiredGripperPoseCallback, this
  );
  user_desired_gripper_status_subscriber_ = nh_.subscribe(
      "/marslite_control/user_desired_gripper_status", 1,
      &SharedControl::userDesiredGripperStatusCallback, this
  );
}

void SharedControl::publishIntentBeliefVisualization() {
  visualization_msgs::MarkerArray belief_visualization
      = intent_inference_.getBeliefVisualization();
  belief_visualization_publisher_.publish(belief_visualization);
}

void SharedControl::publishBlendingGripperPose() {
  geometry_msgs::PoseStamped target_pose = this->getTargetPose();
  desired_gripper_pose_publisher_.publish(target_pose);
}

geometry_msgs::PoseStamped SharedControl::getTargetPose() {
  if (intent_inference_.isNotLocked()) {
    if (is_locked_) {
      // LOCK to UNLOCK transition
      std_msgs::Bool reset_pose_signal_;
      reset_pose_signal_publisher_.publish(reset_pose_signal_);
    }
    is_locked_ = false;
    return user_desired_gripper_pose_;
  }
  is_locked_ = true;
    
  geometry_msgs::Point most_likely_goal_position
      = intent_inference_.getMostLikelyGoalPosition();
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position = most_likely_goal_position;
  // TODO: orientation set to object orientation
  target_pose.pose.orientation = current_gripper_pose_.pose.orientation;
  return target_pose;
}

void SharedControl::currentGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_gripper_pose_ = *msg;
  // TODO: consider synchronization issue (w/ userDesiredGripperPoseCallback)
  intent_inference_.setGripperPosition(current_gripper_pose_.pose.position);
}

void SharedControl::detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& objects) {
  if (begin_recording_) {
    intent_inference_.setRecordedObjects(*objects);
    begin_recording_ = false;
    ROS_INFO_STREAM("Received " << objects->objects.size() << " detected objects.");
  }
}

void SharedControl::recordSignalCallback(const std_msgs::Bool::ConstPtr& signal) {
  begin_recording_ = true;
}

void SharedControl::userDesiredGripperPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  user_desired_gripper_pose_ = *msg;

  // TODO: improve direction estimation
  // TODO: consider synchronization issue (w/ currentGripperPoseCallback)
  controller_direction_estimator_.addwaypoint(user_desired_gripper_pose_.pose.position);
  geometry_msgs::Point user_command_direction
      = controller_direction_estimator_.getAveragedDirection();
  intent_inference_.setUserCommandDirection(user_command_direction);
}

void SharedControl::userDesiredGripperStatusCallback(
    const std_msgs::Bool::ConstPtr& user_desired_gripper_status) {
  intent_inference_.setGripperStatus(*user_desired_gripper_status);
  // Directly publish the user's desired gripper status becuase we don't 
  //   need to process it through shared control.
  desired_gripper_status_publisher_.publish(*user_desired_gripper_status);  
}