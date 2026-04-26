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
    : nh_(nh), loop_rate_(ros::Rate(50)), begin_recording_(false) {
  intent_inference_.parseParameters(nh_);
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
    this->publishRobotState();
  }
  
  this->publishBlendedGripperPose();
  this->publishObjectsWithBelief();
  this->publishIntentBeliefVisualization();
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void SharedControl::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("use_sim", use_sim_, false);
  pnh.param("shared_control_enabled", shared_control_enabled_, true);

  int experiment_id = 1;
  pnh.param("experiment_id", experiment_id, 1);
  intent_inference_.setExperimentID(experiment_id);

  // Arbitration parameters (defaults match the old compile-time constants)
  nh_.param("arbitration/repulsive_force_weak_gain",        repulsive_force_weak_gain_,        0.8);
  nh_.param("arbitration/repulsive_force_strong_gain",      repulsive_force_strong_gain_,      0.2);
  nh_.param("arbitration/repulsive_force_junction_distance", repulsive_force_junction_distance_, 0.3);
  nh_.param("arbitration/attract_max_distance",             attract_max_distance_,             0.40);
  nh_.param("arbitration/attract_gain_horizontal",          attract_gain_horizontal_,          0.8);
  nh_.param("arbitration/attract_gain_vertical",            attract_gain_vertical_,            0.20);
  nh_.param("arbitration/return_assist_angular_rate",       return_assist_angular_rate_,       0.08);
  nh_.param("arbitration/return_assist_radial_rate",       return_assist_radial_rate_,        0.03);
  nh_.param("arbitration/return_assist_vertical_rate",     return_assist_vertical_rate_,      0.05);
  nh_.param("arbitration/return_assist_orientation_rate",  return_assist_orientation_rate_,   0.06);
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
  if (intent_inference_.isReturnAssistanceActive()) {
    // Waypoint navigation: command the arm directly toward the staging
    // pose without mixing user input.
    return this->getReturnAssistPose();
  }
  if (!intent_inference_.isPickAssistanceActive()) {
    return user_desired_gripper_pose_;
  }

  geometry_msgs::PoseStamped blended_pose;
  blended_pose.header.frame_id = user_desired_gripper_pose_.header.frame_id;
  blended_pose.pose.position = this->getBlendedPosition();
  blended_pose.pose.orientation = this->getBlendedOrientation();
  blended_pose.header.stamp = ros::Time::now();
  return blended_pose;
}

geometry_msgs::Point SharedControl::getBlendedPosition() {
  // Arbitration frame: target-centered cylindrical basis (parallel to
  // ground). u_r points horizontally from the current gripper toward the
  // target; u_phi is the lateral axis; u_z is world up. This is the natural
  // frame for funneling the gripper into the target.
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

  const Eigen::Vector3d user_delta = user_command_position - gripper_position;
  const Eigen::Vector3d to_target_xy(
      target_position.x() - gripper_position.x(),
      target_position.y() - gripper_position.y(),
      0.0
  );
  const double d_xy = to_target_xy.norm();
  if (d_xy < kDistanceTolerance) {
    return user_desired_gripper_pose_.pose.position;
  }

  const Eigen::Vector3d u_r = to_target_xy / d_xy;
  const Eigen::Vector3d u_z(0.0, 0.0, 1.0);
  const Eigen::Vector3d u_phi = u_z.cross(u_r);

  // Project the user's delta onto the basis.
  const double v_r   = user_delta.dot(u_r);
  const double v_phi = user_delta.dot(u_phi);
  const double v_z   = user_delta.dot(u_z);

  // Lateral suppression: smoothly interpolate between strong (close) and
  // weak (far) using horizontal distance to the target. Avoids the abrupt
  // step-change of the previous two-bucket version.
  const double t_lat = std::clamp(d_xy / repulsive_force_junction_distance_, 0.0, 1.0);
  const double k_phi = repulsive_force_strong_gain_
      + (repulsive_force_weak_gain_ - repulsive_force_strong_gain_) * t_lat;

  // User motion is passed through on the radial/vertical axes (the teleop
  // side already handles its own scaling — no second amplification here).
  // Only the lateral axis is shaped by the repulsive gain.
  const Eigen::Vector3d shaped_user_delta =
      v_r * u_r + (k_phi * v_phi) * u_phi + v_z * u_z;

  // Position-level attractive lead: pull the blended command a fraction of
  // the *remaining* error toward the target. Two important properties:
  //   1. The lead is proportional to distance, so it is large when far and
  //      vanishes as the gripper arrives (no overshoot, no manual shutoff).
  //   2. It is a position offset, not a per-tick velocity increment. Each
  //      tick the command is still "alpha * d ahead" of the current gripper,
  //      so the arm sees a sustained lead regardless of how slowly it
  //      tracks. The closing speed is therefore set by the combination of
  //      alpha and the arm's own bandwidth, not clipped to bandwidth * step.
  // Logarithmic attract profile: lead = G * D * log(1 + k*t) / log(1 + k),
  // where t = d_xy / D. The old quadratic (G * d^2 / D) dropped to 25% at
  // half-distance; this curve still provides ~74% there, giving the operator
  // noticeable assist throughout the approach without overshooting.
  constexpr double kLogCurvature = 9.0;
  const double t_attract = std::clamp(d_xy / attract_max_distance_, 0.0, 1.0);
  const double log_factor = std::log(1.0 + kLogCurvature * t_attract)
                          / std::log(1.0 + kLogCurvature);
  const double dz = target_position.z() - gripper_position.z();

  const Eigen::Vector3d attract_lead =
      (attract_gain_horizontal_ * attract_max_distance_ * log_factor) * u_r +
      (attract_gain_vertical_   * dz * log_factor) * u_z;

  const Eigen::Vector3d blended_gripper_position =
      gripper_position + shaped_user_delta + attract_lead;

  geometry_msgs::Point blended_position;
  blended_position.x = blended_gripper_position.x();
  blended_position.y = blended_gripper_position.y();
  blended_position.z = blended_gripper_position.z();
  return blended_position;
}

geometry_msgs::Quaternion SharedControl::getBlendedOrientation() {
  // During PICK_ASSIST the operator retains full orientation authority.
  // RETURN_ASSIST is handled separately by getReturnAssistPose().
  return user_desired_gripper_pose_.pose.orientation;
}

geometry_msgs::PoseStamped SharedControl::getReturnAssistPose() {
  // Cylindrical waypoint navigation around tm_base origin:
  //   1. Rotate azimuth θ toward staging angle (fast — primary motion)
  //   2. Contract radius r toward staging radius (slow)
  //   3. Adjust height z toward staging height
  // Each component decays exponentially at its own rate.
  const geometry_msgs::Pose staging = intent_inference_.getStagingPose();
  if (staging == geometry_msgs::Pose())
    return current_gripper_pose_;
  const auto& cur = current_gripper_pose_.pose;

  // --- Current position in cylindrical coords (tm_base origin) ---
  const double r_cur     = std::hypot(cur.position.x, cur.position.y);
  const double theta_cur = std::atan2(cur.position.y, cur.position.x);
  const double z_cur     = cur.position.z;

  // --- Staging position in cylindrical coords ---
  const double r_stg     = std::hypot(staging.position.x, staging.position.y);
  const double theta_stg = std::atan2(staging.position.y, staging.position.x);
  const double z_stg     = staging.position.z;

  // --- Shortest angular path (wrap to [-π, π]) ---
  double dtheta = theta_stg - theta_cur;
  if (dtheta >  M_PI) dtheta -= 2.0 * M_PI;
  if (dtheta < -M_PI) dtheta += 2.0 * M_PI;

  // --- Exponential decay per component ---
  const double theta_cmd = theta_cur + return_assist_angular_rate_  * dtheta;
  const double r_cmd     = r_cur     + return_assist_radial_rate_   * (r_stg - r_cur);
  const double z_cmd     = z_cur     + return_assist_vertical_rate_ * (z_stg - z_cur);

  // --- Back to Cartesian ---
  geometry_msgs::Point pos;
  pos.x = r_cmd * std::cos(theta_cmd);
  pos.y = r_cmd * std::sin(theta_cmd);
  pos.z = z_cmd;

  // --- Orientation: slerp at its own rate ---
  tf2::Quaternion q_cur(cur.orientation.x, cur.orientation.y,
                        cur.orientation.z, cur.orientation.w);
  tf2::Quaternion q_stg(staging.orientation.x, staging.orientation.y,
                        staging.orientation.z, staging.orientation.w);
  if (q_cur.dot(q_stg) < 0.0) {
    q_stg = tf2::Quaternion(-q_stg.x(), -q_stg.y(), -q_stg.z(), -q_stg.w());
  }
  tf2::Quaternion q_cmd = q_cur.slerp(q_stg, return_assist_orientation_rate_).normalized();

  geometry_msgs::PoseStamped result;
  result.header.frame_id = current_gripper_pose_.header.frame_id;
  result.header.stamp = ros::Time::now();
  result.pose.position = pos;
  result.pose.orientation.x = q_cmd.x();
  result.pose.orientation.y = q_cmd.y();
  result.pose.orientation.z = q_cmd.z();
  result.pose.orientation.w = q_cmd.w();
  return result;
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