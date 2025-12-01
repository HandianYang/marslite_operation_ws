#include "shared_control/shared_control.h"

#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

SharedControl::SharedControl(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(10)), begin_recording_(false), 
      intent_inference_(0.1) {
  this->parseParameters();
  this->initializePublishers();
  this->initializeSubscribers();
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void SharedControl::run() {
  while (nh_.ok()) {
    this->updateCurrentGripperPositionForIntentInference();
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

void SharedControl::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("use_sim", use_sim_, false);
}

void SharedControl::initializePublishers() {
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/target_frame", 1
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
  gripper_motion_state_publisher_ = nh_.advertise<std_msgs::String>(
      "/marslite_control/gripper_motion_state", 1
  );
}

void SharedControl::initializeSubscribers() {
  detected_objects_subscriber_ = nh_.subscribe(
      "/yolo/detected_objects", 1,
      &SharedControl::detectedObjectsCallback, this
  );
  record_signal_subscriber_ = nh_.subscribe(
      "/marslite_control/record_signal", 1,
      &SharedControl::recordSignalCallback, this
  );
  position_safety_button_signal_subscriber_ = nh_.subscribe(
      "/marslite_control/position_safety_button_signal", 1,
      &SharedControl::positionSafetyButtonSignalCallback, this
  );
  orientation_safety_button_signal_subscriber_ = nh_.subscribe(
      "/marslite_control/orientation_safety_button_signal", 1,
      &SharedControl::orientationSafetyButtonSignalCallback, this
  );
  user_desired_gripper_pose_subscriber_ = nh_.subscribe(
      "/marslite_control/user_desired_gripper_pose", 1,
      &SharedControl::userDesiredGripperPoseCallback, this
  );
  user_desired_gripper_status_subscriber_ = nh_.subscribe(
      "/marslite_control/user_desired_gripper_status", 1,
      &SharedControl::userDesiredGripperStatusCallback, this
  );
  user_command_velocity_subscriber_ = nh_.subscribe(
      "/marslite_control/user_command_velocity", 1,
      &SharedControl::userCommandVelocityCallback, this
  );
}

void SharedControl::detectedObjectsCallback(
    const detection_msgs::DetectedObjectArray::ConstPtr& objects) {
  if (begin_recording_) {
    intent_inference_.setRecordedObjects(*objects);
    begin_recording_ = false;
  }
}

void SharedControl::recordSignalCallback(
    const std_msgs::Bool::ConstPtr& signal) {
  begin_recording_ = true;
  // TODO: Remove this test code later
  { // test scope
    detection_msgs::DetectedObject object1;
    object1.label = "object1";
    object1.frame = "odom";
    object1.confidence = 1.0;
    object1.centroid.x = 0.378;
    object1.centroid.y = 0.755;
    object1.centroid.z = 0.984;

    detection_msgs::DetectedObject object2;
    object2.label = "object2";
    object2.frame = "odom";
    object2.confidence = 1.0;
    object2.centroid.x = 0.544;
    object2.centroid.y = 0.775;
    object2.centroid.z = 0.968;

    detection_msgs::DetectedObjectArray recorded_objects;
    recorded_objects.objects.push_back(object1);
    recorded_objects.objects.push_back(object2);
    intent_inference_.setRecordedObjects(recorded_objects);
  } // end test scope
}

void SharedControl::positionSafetyButtonSignalCallback(
    const std_msgs::Bool::ConstPtr& signal) {
  position_safety_button_signal_ = *signal;
  intent_inference_.setSafetyButtonStatus(signal->data);
}

void SharedControl::orientationSafetyButtonSignalCallback(
    const std_msgs::Bool::ConstPtr& signal) {
  orientation_safety_button_signal_ = *signal;
}

void SharedControl::userDesiredGripperPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  user_desired_gripper_pose_ = *msg;
}

void SharedControl::userDesiredGripperStatusCallback(
    const std_msgs::Bool::ConstPtr& user_desired_gripper_status) {
  intent_inference_.setGripperStatus(*user_desired_gripper_status);
  // Directly publish the user's desired gripper status becuase we don't 
  //   need to process it through shared control.
  desired_gripper_status_publisher_.publish(*user_desired_gripper_status);
}

void SharedControl::userCommandVelocityCallback(
    const geometry_msgs::Vector3::ConstPtr& user_command_velocity) {
  intent_inference_.setUserCommandVelocity(*user_command_velocity);
}

void SharedControl::updateCurrentGripperPositionForIntentInference() {
  const std::string target_frame = use_sim_ ? "robotiq_85_base_link" : "tm_gripper";
  const geometry_msgs::TransformStamped current_gripper_transform =
      tf2_listener_.lookupTransform("tm_base", target_frame);
  
  geometry_msgs::Point current_gripper_position;
  current_gripper_position.x = current_gripper_transform.transform.translation.x;
  current_gripper_position.y = current_gripper_transform.transform.translation.y;
  current_gripper_position.z = current_gripper_transform.transform.translation.z;
  intent_inference_.setGripperPosition(current_gripper_position);
}

void SharedControl::publishIntentBeliefVisualization() {
  visualization_msgs::MarkerArray belief_visualization
      = intent_inference_.getBeliefVisualization();
  belief_visualization_publisher_.publish(belief_visualization);
}

void SharedControl::publishBlendingGripperPose() {
  geometry_msgs::PoseStamped target_pose = this->getTargetPose();
  if (target_pose.header.frame_id.empty())  return;
  desired_gripper_pose_publisher_.publish(target_pose);
}

geometry_msgs::PoseStamped SharedControl::getTargetPose() {
  if (intent_inference_.isInLockedState()) {
    is_previously_locked_ = true;
    
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "tm_base";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position = this->getTargetPosition();
    target_pose.pose.orientation = this->getTargetOrientation();
    return target_pose;
  }

  if (is_previously_locked_) {
    // LOCK to UNLOCK transition
    std_msgs::Bool reset_pose_signal_;
    reset_pose_signal_publisher_.publish(reset_pose_signal_);
  }
  is_previously_locked_ = false;
  return user_desired_gripper_pose_;
}

geometry_msgs::Point SharedControl::getTargetPosition() {
  return intent_inference_.getPlannedPosition();
}

geometry_msgs::Quaternion SharedControl::getTargetOrientation() {
  geometry_msgs::Vector3 target_direction = intent_inference_.getTargetDirection();
  RPY target_orientation;
  target_orientation.roll = M_PI / 2;
  target_orientation.pitch = 0;
  target_orientation.yaw = M_PI / 2 + std::atan2(target_direction.y, target_direction.x);
  return target_orientation.convertToQuaternion();
}