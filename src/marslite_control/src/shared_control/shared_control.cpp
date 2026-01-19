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
    : nh_(nh), loop_rate_(ros::Rate(50)), begin_recording_(false), 
      intent_inference_(0.1) {
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

void SharedControl::runInference() {
  while (nh_.ok()) {
    this->runInferenceOnce();
    loop_rate_.sleep();
    ros::spinOnce();
  }
}

void SharedControl::runInferenceOnce() {
  // While `shared_control_enabled_` is false, the system will only update the
  //  object positions to tm_base frame. Although `publishObjectsWithBelief()`
  //  is called, the belief value will remain default (i.e., uniform distribution).
  //
  // The command to gripper pose will always be published, either blended or
  //   directly from user input.
  
  intent_inference_.updateObjectPositionToTmBase();
  if (shared_control_enabled_) {
    intent_inference_.updateGripperPosition();
    intent_inference_.updateRobotState();
    intent_inference_.updateBelief();
    this->publishIntentBeliefVisualization();
    this->publishRobotState();
  }
  
  this->publishBlendedGripperPose();
  this->publishObjectsWithBelief();
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void SharedControl::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("use_sim", use_sim_, false);
  pnh.param("shared_control_enabled", shared_control_enabled_, true);
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
  current_gripper_pose_subscriber_ = nh_.subscribe(
      "/marslite_control/gripper_pose", 1,
      &SharedControl::currentGripperPoseCallback, this
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
    ROS_INFO_STREAM(objects->objects.size() << " objects recorded");
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

void SharedControl::currentGripperPoseCallback(
    const geometry_msgs::PoseStamped& pose) {
  current_gripper_pose_ = pose;
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

void SharedControl::publishBlendedGripperPose() {
  geometry_msgs::PoseStamped target_pose = this->getBlendedPose();
  if (target_pose.header.frame_id.empty())  return;
  desired_gripper_pose_publisher_.publish(target_pose);
}

geometry_msgs::PoseStamped SharedControl::getBlendedPose() {
  if (!intent_inference_.isPickAssistanceActive())
    return user_desired_gripper_pose_;

  geometry_msgs::PoseStamped blended_pose;
  blended_pose.header.frame_id = user_desired_gripper_pose_.header.frame_id;
  blended_pose.pose.position = this->getBlendedPosition();
  blended_pose.pose.orientation = this->getBlendedOrientation();
  blended_pose.header.stamp = ros::Time::now();
  return blended_pose;
}

geometry_msgs::Point SharedControl::getBlendedPosition() {
  /// TODO: Refine the code
  const Eigen::Vector3d user_command_position(
      user_desired_gripper_pose_.pose.position.x,
      user_desired_gripper_pose_.pose.position.y,
      user_desired_gripper_pose_.pose.position.z
  );
  const Eigen::Vector3d gripper_position(
      current_gripper_pose_.pose.position.x,
      current_gripper_pose_.pose.position.y,
      current_gripper_pose_.pose.position.z
  );
  const geometry_msgs::Point tp = intent_inference_.getTargetPosition();
  const Eigen::Vector3d target_position(tp.x, tp.y, tp.z);

  const Eigen::Vector3d user_desired_direction = user_command_position - gripper_position;
  Eigen::Vector3d target_direction = target_position - gripper_position;
  target_direction.z() = 0; // to make `u_r` and `u_phi` always parallel to the ground
  const double target_distance = target_direction.norm();
  if (target_distance < kDistanceTolerance)
    return user_desired_gripper_pose_.pose.position;
  
  /// 3 axes in unit vector
  Eigen::Vector3d u_r = target_direction.normalized();
  Eigen::Vector3d u_z(0, 0, 1);
  Eigen::Vector3d u_phi = u_z.cross(u_r);

  /// user command's projection to 3 axes 
  const double v_r = user_desired_direction.dot(u_r);
  const double v_z = user_desired_direction.dot(u_z);
  const double v_phi = user_desired_direction.dot(u_phi);
  
  /// gains
  // Set k_r >= 1.0 to enhance the attractive force, and set it to 1.0 
  //  for retreat movements (no interference)
  const double k_r = (v_r >= 0) ? kAttractiveForceGain : 1.0;
  // No interference (a.k.a. let user decide)
  const double k_z = 1.0;
  // Set k_phi < 1.0 to restrict lateral movements
  const double k_phi = (target_distance > kRepulsiveForceJunctionDistance) ?
      kRepulsiveForceWeakGain : kRepulsiveForceStrongGain;

  const Eigen::Vector3d blended_displacement = (k_r * v_r) * u_r + 
      (k_z * v_z) * u_z + 
      (k_phi * v_phi) * u_phi;
  const Eigen::Vector3d blended_gripper_position = gripper_position + blended_displacement;
  
  geometry_msgs::Point blended_position;
  blended_position.x = blended_gripper_position.x();
  blended_position.y = blended_gripper_position.y();
  blended_position.z = blended_gripper_position.z();
  return blended_position;
}

geometry_msgs::Quaternion SharedControl::getBlendedOrientation() {
  /// TODO: Modify this function
  RPY user_rpy; 
  user_rpy.convertFromQuaternion(user_desired_gripper_pose_.pose.orientation);

  RPY target_rpy;
  target_rpy.roll = M_PI / 2;
  target_rpy.pitch = 0;
  target_rpy.yaw = user_rpy.yaw;
  tf2::Quaternion q_target = target_rpy.convertToTF2Quaternion();
  tf2::Quaternion q_user;
  tf2::fromMsg(user_desired_gripper_pose_.pose.orientation, q_user);

  const double rot_gain = 0.1;
  tf2::Quaternion q_final = q_user.slerp(q_target, rot_gain);
  q_final.normalize();

  geometry_msgs::Quaternion blended_orientation;
  blended_orientation = tf2::toMsg(q_final);
  return blended_orientation;
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