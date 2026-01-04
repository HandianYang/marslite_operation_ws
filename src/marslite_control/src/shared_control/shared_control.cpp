#include "shared_control/shared_control.h"

#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

#include "detection_msgs/RobotState.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

SharedControl::SharedControl(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(10)), begin_recording_(false), 
      intent_inference_(0.1),
      is_pick_assistance_active_(false) {
  this->parseParameters();
  this->initializePublishers();
  this->initializeSubscribers();
  reset_client_ = nh_.serviceClient<std_srvs::Trigger>("reset_teleop_origin");
  intent_inference_.setUseSim(use_sim_);
  intent_inference_.registerResetPoseHandler([this]() -> bool {
    return this->callResetPoseService();
  });
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

bool SharedControl::callResetPoseService() {
  std_srvs::Trigger srv;
  if (reset_client_.call(srv)) {
    return srv.response.success;
  } else {
    ROS_ERROR("Failed to call reset_teleop_origin service");
    return false;
  }
}

void SharedControl::run_inference() {
  while (nh_.ok()) {
    this->run_inference_once();
    ros::Rate(10).sleep();
    ros::spinOnce();
  }
}

void SharedControl::run_inference_once() {
  /// TODO: Specify the function according to the task

  /// Step 1. Initialization
  intent_inference_.updateGripperPosition();
  intent_inference_.updateObjectPositionToTmBase();
  intent_inference_.updateRobotState();

  /// Step 2. Update belief
  intent_inference_.updateBelief();

  /// Step 3. Publish results
  this->publishBlendingGripperPose();
  // Publish for visualization
  this->publishIntentBeliefVisualization();
  // Publish for rosbag
  this->publishObjectsWithBelief();
  this->publishRobotState();
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
  robot_state_publisher_ = nh_.advertise<detection_msgs::RobotState>(
      "/marslite_control/robot_state", 1
  );
  objects_with_belief_publisher_ = nh_.advertise<detection_msgs::DetectedObjectArray>(
      "/marslite_control/objects_with_belief", 1
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
  user_command_velocity_ = *user_command_velocity;
}

void SharedControl::publishBlendingGripperPose() {
  geometry_msgs::PoseStamped target_pose = this->getTargetPose();
  if (target_pose.header.frame_id.empty())  return;
  desired_gripper_pose_publisher_.publish(target_pose);
}

void SharedControl::publishIntentBeliefVisualization() {
  visualization_msgs::MarkerArray belief_visualization
      = intent_inference_.getBeliefVisualization();
  belief_visualization_publisher_.publish(belief_visualization);
}

void SharedControl::publishRobotState() {
  detection_msgs::RobotState robot_state_msg;
  robot_state_msg.id = static_cast<int8_t>(intent_inference_.getRobotState());
  robot_state_msg.name = toString(intent_inference_.getRobotState());
  robot_state_publisher_.publish(robot_state_msg);
}

void SharedControl::publishObjectsWithBelief() {
  detection_msgs::DetectedObjectArray objects_with_belief
      = intent_inference_.getObjectsWithBelief();
  objects_with_belief_publisher_.publish(objects_with_belief);
}

geometry_msgs::PoseStamped SharedControl::getTargetPose() {
  static geometry_msgs::Point target_position;
  static geometry_msgs::Quaternion target_orientation;
  if (intent_inference_.isPickAssistanceActive()) {
    if (is_pick_assistance_active_ == false) {
      target_position = this->getTargetPosition();
      target_orientation = this->getTargetOrientation();
      is_pick_assistance_active_ = true;
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "tm_base";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position = target_position;
    target_pose.pose.orientation = target_orientation;

    return target_pose;
  }

  is_pick_assistance_active_ = false;
  return user_desired_gripper_pose_;
}

geometry_msgs::Point SharedControl::getTargetPosition() {
  return intent_inference_.getTargetPosition();
}

geometry_msgs::Quaternion SharedControl::getTargetOrientation() {
  geometry_msgs::Vector3 target_direction = intent_inference_.getTargetDirection();
  RPY target_orientation;
  target_orientation.roll = M_PI / 2;
  target_orientation.pitch = 0;
  target_orientation.yaw = M_PI / 2 + std::atan2(target_direction.y, target_direction.x);
  return target_orientation.convertToQuaternion();
}