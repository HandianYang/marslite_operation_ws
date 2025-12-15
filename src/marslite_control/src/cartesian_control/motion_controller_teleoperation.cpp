#include "cartesian_control/motion_controller_teleoperation.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <cmath>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

MotionControllerTeleoperation::MotionControllerTeleoperation(const ros::NodeHandle& nh)
    : nh_(nh), loop_rate_(10),
      is_position_change_enabled_(false),
      is_orientation_change_enabled_(false) {
  this->parseParameters();
  this->initializePublishers();
  this->initializeSubscribers();
  user_command_velocity_estimator_.setBufferSize(kEstimatorBufferSize);
  user_command_velocity_estimator_.setMinSpeed(kEstimatorMinSpeed);
  gripper_velocity_estimator_.setBufferSize(kEstimatorBufferSize);
  gripper_velocity_estimator_.setMinSpeed(kEstimatorMinSpeed);
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

geometry_msgs::PoseStamped MotionControllerTeleoperation::generateInitialGripperPose(
    const double& control_view_angle) {
  // Pre-defined gripper pose parameters:
  const double kGripperPoseRadius  = 0.538;  // [m]
  const double kGripperPoseYOffset = -0.122; // [m]
  const double kGripperPoseHeight  = 0.666;  // [m]
  const double kGripperPoseRoll  = use_sim_ ? 0.0 : M_PI / 2; // [rad]
  const double kGripperPosePitch = 0.000; // [rad]
  const double kGripperPoseYaw   = control_view_angle + control_view_offset_; // [rad]

  geometry_msgs::PoseStamped initial_pose;
  initial_pose.header.frame_id = "tm_base";
  initial_pose.pose.position.x =
      kGripperPoseRadius * cos(control_view_angle) - 
      kGripperPoseYOffset * sin(control_view_angle);
  initial_pose.pose.position.y = 
      kGripperPoseRadius  * sin(control_view_angle) + 
      kGripperPoseYOffset * cos(control_view_angle);
  initial_pose.pose.position.z = kGripperPoseHeight;

  RPY initial_pose_rpy;
  initial_pose_rpy.roll = kGripperPoseRoll;
  initial_pose_rpy.pitch = kGripperPosePitch;
  initial_pose_rpy.yaw = kGripperPoseYaw;
  while (initial_pose_rpy.yaw > M_PI)  initial_pose_rpy.yaw -= 2 * M_PI;
  while (initial_pose_rpy.yaw < -M_PI) initial_pose_rpy.yaw += 2 * M_PI;
  initial_pose.pose.orientation = initial_pose_rpy.convertToQuaternion();
  
  return initial_pose;
}


void MotionControllerTeleoperation::teleoperateToPose(const geometry_msgs::PoseStamped& target_pose) {
  initial_gripper_pose_ = desired_gripper_pose_ = target_pose;
  target_frame_publisher_.publish(target_pose);
  this->calculateCurrentGripperPose();
  
  ROS_INFO_STREAM("Guiding to initial gripper pose: \n"
      << " - Position: (" 
      << target_pose.pose.position.x << ", "
      << target_pose.pose.position.y << ", "
      << target_pose.pose.position.z << ")\n"
      << " - Orientation: ("
      << target_pose.pose.orientation.x << ", "
      << target_pose.pose.orientation.y << ", "
      << target_pose.pose.orientation.z << ", "
      << target_pose.pose.orientation.w << ")"
  );

  // Perform teleoperation until the current gripper pose reaches the target pose
  while (nh_.ok() && !this->targetPoseIsReached(target_pose)) {
    ROS_INFO_STREAM_THROTTLE(5, "Waiting to reach the target pose...");
    this->calculateCurrentGripperPose();
    loop_rate_.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Reached the target pose. You can start teleoperation now!");
}

void MotionControllerTeleoperation::run() {
  this->initializeGripperPose();
  this->initializeLeftControllerPose();
  while (nh_.ok()) {
    this->calculateCurrentGripperPose();
    {
      std::lock_guard<std::mutex> lock(desired_gripper_pose_mutex_);
      if (this->isAnySafetyButtonPressed()) {
        this->calculateDesiredGripperPose();
        this->calculateGripperVelocity();
        this->calculateGripperVelocityMarker();
        this->calculateUserCommandVelocity();
        this->calculateUserCommandVelocityMarker();
      } else {
        this->initializeGripperPose();
        this->initializeLeftControllerPose();
      }

      this->publishDesiredGripperPose();
      this->publisherGripperVelocity();
      this->publishGripperVelocityMarker();
      this->publishUserCommandVelocity();
      this->publishUserCommandVelocityMarker();
    } // end desired_gripper_pose_mutex_ scope
    this->publishMobilePlatformVelocity();

    loop_rate_.sleep();
    ros::spinOnce();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void MotionControllerTeleoperation::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("position_scale", position_scale_, 0.8);
  pnh.param("orientation_scale", orientation_scale_, 0.8);
  pnh.param("linear_velocity_scale", linear_velocity_scale_, 0.15);
  pnh.param("angular_velocity_scale", angular_velocity_scale_, 0.15);
  pnh.param("use_shared_controller", use_shared_controller_, false);
  pnh.param("use_sim", use_sim_, false);

  // TF yaw offset from control view direction
  // [Note] TF is defined as: /tm_base -> /tm_gripper
  // [Note] Actual_yaw = control_view_angle_ + control_view_offset_
  control_view_offset_ = use_sim_ ? 0.0 : M_PI / 2;

  ROS_INFO_STREAM(std::boolalpha << "Parameters: " 
      << "\n * control_view_offset: " << control_view_offset_
      << "\n * position_scale: " << position_scale_
      << "\n * orientation_scale: " << orientation_scale_
      << "\n * linear_velocity_scale: " << linear_velocity_scale_
      << "\n * angular_velocity_scale: " << angular_velocity_scale_
      << "\n * use_shared_controller: " << use_shared_controller_
      << "\n * use_sim: " << use_sim_
  );
}

void MotionControllerTeleoperation::initializePublishers() {
  // This publisher is used only in teleoperateToPose() to publish the target
  //   pose command to the robot directly.
  target_frame_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/target_frame", 1);
  
  // The desired gripper pose will be published to different topics according
  // to the `use_shared_controller_` parameter.
  // - use_shared_controller_ = true: published to shared controller
  // - use_shared_controller_ = false: published to the robot
  const std::string gripper_pose_topic = use_shared_controller_ ?
      "/marslite_control/user_desired_gripper_pose" :
      "/target_frame";
  const std::string gripper_status_topic = use_shared_controller_ ?
      "/marslite_control/user_desired_gripper_status" :
      "/gripper/cmd_gripper";
  
  gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(gripper_pose_topic, 1);
  gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(gripper_status_topic, 1);
  position_safety_button_signal_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/marslite_control/position_safety_button_signal", 1);
  orientation_safety_button_signal_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/marslite_control/orientation_safety_button_signal", 1);
  mobile_platform_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(
      "/mob_plat/cmd_vel", 1);
  record_signal_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/marslite_control/record_signal", 1);
  
  user_command_velocity_publisher_ = nh_.advertise<geometry_msgs::Vector3>(
      "/marslite_control/user_command_velocity", 1);
  user_command_velocity_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "/marslite_control/user_command_velocity_marker", 1);
  gripper_velocity_publisher_ = nh_.advertise<geometry_msgs::Vector3>(
      "/marslite_control/gripper_velocity", 1);
  gripper_velocity_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "/marslite_control/gripper_velocity_marker", 1);
}

void MotionControllerTeleoperation::initializeSubscribers() {
  left_controller_pose_subscriber_ = nh_.subscribe(
      "/unity/controller/left/pose", 1,
      &MotionControllerTeleoperation::leftControllerPoseCallback, this
  );
  left_controller_joy_subscriber_ = nh_.subscribe(
      "/unity/controller/left/joy", 1,
      &MotionControllerTeleoperation::leftControllerJoyCallback, this
  );
  reset_pose_signal_subscriber_ = nh_.subscribe(
      "/marslite_control/lock_state_signal", 1,
      &MotionControllerTeleoperation::resetPoseSignalCallback, this
  );
}

void MotionControllerTeleoperation::leftControllerPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_left_controller_pose_.header.frame_id = "tm_base";
  current_left_controller_pose_.header.stamp = ros::Time::now();
  current_left_controller_pose_.pose = msg->pose;
}

void MotionControllerTeleoperation::leftControllerJoyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  switch (msg->axes.size()) {
    case 4:
      // [3] primary hand trigger: Enable/disable orientational change
      is_orientation_change_enabled_ = (msg->axes[3] > kTriggerThreshold);
      orientation_safety_button_signal_.data = is_orientation_change_enabled_;
      orientation_safety_button_signal_publisher_.publish(orientation_safety_button_signal_);
    case 3:
      // [2] primary index trigger: Enable/disable positional change
      is_position_change_enabled_ = (msg->axes[2] > kTriggerThreshold);
      position_safety_button_signal_.data = is_position_change_enabled_;
      position_safety_button_signal_publisher_.publish(position_safety_button_signal_);
    case 2:
      // [1] horizontal axis: Turn left/right
      mobile_platform_velocity_.angular.z = std::abs(msg->axes[1]) > kAnalogJoystickDeadzone ?
          msg->axes[1] * angular_velocity_scale_ : 0.0;
    case 1:
      // [0] vertical axis: Move forward/backward
      mobile_platform_velocity_.linear.x = std::abs(msg->axes[0]) > kAnalogJoystickDeadzone ?
          msg->axes[0] * linear_velocity_scale_ : 0.0;
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick axes (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }

  switch (msg->buttons.size()) {
    case 2:
      // [1] Y button: Send `record_signal` to `shared_control`
      if (msg->buttons[1] == 1) {
        record_signal_.data = true;
        record_signal_publisher_.publish(record_signal_);
      }
    case 1:
      // [0] X button: Close/Open the gripper
      if (msg->buttons[0] == 1) {
        this->toggleGripperStatus();
      }
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick buttons (%lu).", msg->buttons.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }
}

void MotionControllerTeleoperation::toggleGripperStatus() {
  static ros::Time last_toggle_time = ros::Time(0);
  ros::Time current_time = ros::Time::now();
  if ((current_time - last_toggle_time).toSec() > kGripperToggleCooldownTime) {
    desired_gripper_status_.data = !desired_gripper_status_.data;
    if (desired_gripper_status_.data) {
      ROS_INFO("Close the gripper");
    } else {
      ROS_INFO("Open the gripper");
    }
    gripper_status_publisher_.publish(desired_gripper_status_);
    last_toggle_time = current_time;
  }
}

void MotionControllerTeleoperation::resetPoseSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(desired_gripper_pose_mutex_);
    this->initializeGripperPose();
    this->initializeLeftControllerPose();
  } // end mutex scope
}

const bool MotionControllerTeleoperation::targetPoseIsReached(
    const geometry_msgs::PoseStamped& target_pose) const {
  const double dx = current_gripper_pose_.pose.position.x - target_pose.pose.position.x;
  const double dy = current_gripper_pose_.pose.position.y - target_pose.pose.position.y;
  const double dz = current_gripper_pose_.pose.position.z - target_pose.pose.position.z;
  const double position_error = sqrt(dx*dx + dy*dy + dz*dz);
  // ROS_INFO_STREAM_THROTTLE(1, "Position error: " << position_error);
  if (position_error > kPositionTolerance)  return false;

  auto toEig = [](const geometry_msgs::Quaternion& q){
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  };
  Eigen::Quaterniond q1 = toEig(current_gripper_pose_.pose.orientation);
  Eigen::Quaterniond q2 = toEig(target_pose.pose.orientation);
  const double n1 = q1.norm(), n2 = q2.norm();
  if (n1 < 1e-12 || n2 < 1e-12) return false;
  q1.normalize(); q2.normalize();

  // Smallest angular distance between orientations.
  // |dot| accounts for the double-cover (q and -q).
  const double dot = std::abs(q1.dot(q2));
  // Numerical clamp: acos argument must be in [0,1].
  auto clamp01 = [](double x) { return std::max(0.0, std::min(1.0, x)); };
  const double orientation_error = 2.0 * std::acos(clamp01(dot));

  return orientation_error <= kOrientationTolerance;
}

void MotionControllerTeleoperation::initializeGripperPose() {
  this->resetPositionalMovement();
  this->resetOrientationalMovement();
  initial_gripper_pose_ = desired_gripper_pose_;
  initial_gripper_cylindrical_pose_ = desired_gripper_cylindrical_pose_
                                    = current_gripper_cylindrical_pose_;
}

void MotionControllerTeleoperation::resetPositionalMovement() {
  // Shift a little to avoid immediate stop
  gripper_velocity_estimator_.estimateVelocity();
  gripper_velocity_ = gripper_velocity_estimator_.getEstimatedVelocity();
  desired_gripper_pose_.pose.position.x = current_gripper_pose_.pose.position.x + gripper_velocity_.x * 0.01;
  desired_gripper_pose_.pose.position.y = current_gripper_pose_.pose.position.y + gripper_velocity_.y * 0.01;
  desired_gripper_pose_.pose.position.z = current_gripper_pose_.pose.position.z + gripper_velocity_.z * 0.01;
  gripper_velocity_ = geometry_msgs::Vector3();

  // reset user_command_velocity_
  user_command_velocity_estimator_.clear();
  user_command_velocity_ = geometry_msgs::Vector3(); 
}

void MotionControllerTeleoperation::resetOrientationalMovement() {
  desired_gripper_pose_.pose.orientation = current_gripper_pose_.pose.orientation;
}

void MotionControllerTeleoperation::initializeLeftControllerPose() {
  initial_left_controller_pose_ = current_left_controller_pose_;
}

void MotionControllerTeleoperation::calculateCurrentGripperPose() {
  const std::string source_frame = use_sim_ ? "robotiq_85_base_link" : "tm_gripper";
  current_gripper_pose_ = tf2_listener_.lookupTransform<geometry_msgs::PoseStamped>("tm_base", source_frame);

  RPY gripper_rpy;  gripper_rpy.convertFromPoseStamped(current_gripper_pose_);
  current_gripper_cylindrical_pose_.yaw = gripper_rpy.yaw;
  const double control_view_angle = gripper_rpy.yaw - control_view_offset_;
  current_gripper_cylindrical_pose_.radius = 
      current_gripper_pose_.pose.position.x * std::cos(control_view_angle) +
      current_gripper_pose_.pose.position.y * std::sin(control_view_angle);
  current_gripper_cylindrical_pose_.height = current_gripper_pose_.pose.position.z;

  ROS_INFO_STREAM_THROTTLE(1, "- Radius/Yaw/Height: "
      << current_gripper_cylindrical_pose_.radius << " [m] / "
      << current_gripper_cylindrical_pose_.yaw * 180 / M_PI << " [deg] / "
      << current_gripper_cylindrical_pose_.height << " [m]"
  );
}

void MotionControllerTeleoperation::calculateGripperVelocity() {
  geometry_msgs::PointStamped waypoint;
  waypoint.header = current_gripper_pose_.header;
  waypoint.point = current_gripper_pose_.pose.position;
  gripper_velocity_estimator_.addWaypoint(waypoint);
  gripper_velocity_estimator_.estimateVelocity();
  gripper_velocity_ = gripper_velocity_estimator_.getEstimatedVelocity();
}

void MotionControllerTeleoperation::calculateDesiredGripperPose() {
  desired_gripper_pose_.header.stamp = ros::Time::now();
  if (is_orientation_change_enabled_) {
    this->calculateDesiredGripperOrientation();
  } else {
    this->resetOrientationalMovement();
  }

  if (is_position_change_enabled_) {
    this->calculateDesiredGripperPosition();
  } else {
    this->resetPositionalMovement();
  }
}

void MotionControllerTeleoperation::calculateDesiredGripperOrientation() {
  RPY orientation_difference = this->getOrientationDifference();
  RPY scaled_orientation_difference = this->scaleOrientationDifference(orientation_difference);
  this->applyOrientationDifference(scaled_orientation_difference);
}

RPY MotionControllerTeleoperation::getOrientationDifference() {
  RPY initial_left_controller_rpy;
  initial_left_controller_rpy.convertFromPoseStamped(initial_left_controller_pose_);
  RPY current_left_controller_rpy;
  current_left_controller_rpy.convertFromPoseStamped(current_left_controller_pose_);

  RPY orientation_difference = {
      this->restrictAngleWithinPI(current_left_controller_rpy.roll - initial_left_controller_rpy.roll),
      this->restrictAngleWithinPI(current_left_controller_rpy.pitch - initial_left_controller_rpy.pitch),
      this->restrictAngleWithinPI(current_left_controller_rpy.yaw - initial_left_controller_rpy.yaw)
  };
  return orientation_difference;
}

RPY MotionControllerTeleoperation::scaleOrientationDifference(const RPY& orientation_difference) {
  RPY scaled_orientation_difference;
  scaled_orientation_difference.roll = orientation_difference.roll * orientation_scale_;
  scaled_orientation_difference.pitch = orientation_difference.pitch * orientation_scale_;
  scaled_orientation_difference.yaw = orientation_difference.yaw * orientation_scale_;
  return scaled_orientation_difference;
}

void MotionControllerTeleoperation::applyOrientationDifference(const RPY& scaled_orientation_difference) {
  // [NOTE] The transformation from left controller to `/tm_gripper` to
  //   `/tm_base` is applied in this function:
  //   | controller | /tm_base    |
  //   |------------|-------------|
  //   |   +-roll   |   +-pitch   |
  //   |   +-pitch  |   -+roll    |
  //   |   +-yaw    |   +-yaw     |
  RPY transformed_orientation_difference;
  transformed_orientation_difference.roll = scaled_orientation_difference.pitch;
  transformed_orientation_difference.pitch = 0.0;
  // [NOTE] The pitch of the transformed RPY is disabled because we don't need
  //   roll rotation for the gripper (w.r.t. `tm_base`)`
  // 
  //   Uncomment this line to enable pitch change:
  // transformed_orientation_difference.pitch = -scaled_orientation_difference.roll;
  transformed_orientation_difference.yaw = scaled_orientation_difference.yaw;

  RPY initial_gripper_rpy;
  initial_gripper_rpy.convertFromPoseStamped(initial_gripper_pose_);
  RPY target_gripper_rpy = {
      this->restrictAngleWithinPI(initial_gripper_rpy.roll + transformed_orientation_difference.roll),
      this->restrictAngleWithinPI(initial_gripper_rpy.pitch + transformed_orientation_difference.pitch),
      this->restrictAngleWithinPI(initial_gripper_rpy.yaw + transformed_orientation_difference.yaw)
  };
  desired_gripper_pose_.pose.orientation = target_gripper_rpy.convertToQuaternion();
}

void MotionControllerTeleoperation::calculateDesiredGripperPosition() {
  geometry_msgs::Vector3 position_difference = this->getPositionDifference();
  geometry_msgs::Vector3 scaled_position_difference = this->scalePositionDifference(position_difference);
  this->applyPositionDifference(scaled_position_difference);
}

geometry_msgs::Vector3 MotionControllerTeleoperation::getPositionDifference() {
  geometry_msgs::Vector3 position_difference;
  position_difference.x = current_left_controller_pose_.pose.position.x
      - initial_left_controller_pose_.pose.position.x;
  position_difference.y = current_left_controller_pose_.pose.position.y
      - initial_left_controller_pose_.pose.position.y;
  position_difference.z = current_left_controller_pose_.pose.position.z
      - initial_left_controller_pose_.pose.position.z;
  return position_difference;
}

geometry_msgs::Vector3 MotionControllerTeleoperation::scalePositionDifference(
    const geometry_msgs::Vector3& position_difference) {
  geometry_msgs::Vector3 scaled_position_difference;
  scaled_position_difference.x = position_difference.x * position_scale_;
  scaled_position_difference.y = position_difference.y * position_scale_;
  scaled_position_difference.z = position_difference.z * position_scale_;
  return scaled_position_difference;
}

void MotionControllerTeleoperation::applyPositionDifference(
    const geometry_msgs::Vector3& scaled_position_difference) {
  // TODO: Make kVirtualArmLength adjustable via calibration process
  // TODO: Refactor
  const double kVirtualArmLength = 0.4; // [m]
  const double relative_x = scaled_position_difference.x + kVirtualArmLength;
  const double relative_y = scaled_position_difference.y;

  const double current_user_radius = std::sqrt(relative_x * relative_x + relative_y * relative_y);
  const double delta_radius_input = current_user_radius - kVirtualArmLength;
  const double delta_angle_input = std::atan2(relative_y, relative_x);

  desired_gripper_cylindrical_pose_.radius =
      initial_gripper_cylindrical_pose_.radius + delta_radius_input * 1.0;
  desired_gripper_cylindrical_pose_.yaw =
      initial_gripper_cylindrical_pose_.yaw + delta_angle_input * 1.0;
  desired_gripper_cylindrical_pose_.height =
      initial_gripper_cylindrical_pose_.height + scaled_position_difference.z;
  
  const double control_view_angle = desired_gripper_cylindrical_pose_.yaw - control_view_offset_;
  desired_gripper_pose_.pose.position.x =
      desired_gripper_cylindrical_pose_.radius * std::cos(control_view_angle) - 
      kGripperPoseYOffset * std::sin(control_view_angle);
  desired_gripper_pose_.pose.position.y =
      desired_gripper_cylindrical_pose_.radius * std::sin(control_view_angle) + 
      kGripperPoseYOffset * std::cos(control_view_angle);
  desired_gripper_pose_.pose.position.z = desired_gripper_cylindrical_pose_.height;

  RPY current_gripper_rpy;
  current_gripper_rpy.convertFromPoseStamped(current_gripper_pose_);
  RPY desired_gripper_rpy;
  desired_gripper_rpy.roll = current_gripper_rpy.roll;
  desired_gripper_rpy.pitch = current_gripper_rpy.pitch;
  desired_gripper_rpy.yaw = desired_gripper_cylindrical_pose_.yaw;
  desired_gripper_pose_.pose.orientation = desired_gripper_rpy.convertToQuaternion();
}

void MotionControllerTeleoperation::calculateUserCommandVelocity() {
  geometry_msgs::PointStamped waypoint;
  waypoint.header = desired_gripper_pose_.header;
  waypoint.point = desired_gripper_pose_.pose.position;
  user_command_velocity_estimator_.addWaypoint(waypoint);
  user_command_velocity_estimator_.estimateVelocity();
  user_command_velocity_ = user_command_velocity_estimator_.getEstimatedVelocity();
}

void MotionControllerTeleoperation::calculateUserCommandVelocityMarker() {
  user_command_velocity_marker_.header.frame_id = "tm_base";
  user_command_velocity_marker_.header.stamp = ros::Time::now();
  user_command_velocity_marker_.ns = "user_command_velocity";
  user_command_velocity_marker_.id = 0;
  user_command_velocity_marker_.type = visualization_msgs::Marker::ARROW;

  const tf2::Vector3 user_command_velocity_tf2(
      user_command_velocity_.x,
      user_command_velocity_.y,
      user_command_velocity_.z
  );
  const double user_command_speed = user_command_velocity_tf2.length();
  if (user_command_speed < 1e-3) {
    user_command_velocity_marker_.action = visualization_msgs::Marker::DELETE;
    return;
  }
  tf2::Vector3 user_command_velocity_normalized = 
      user_command_velocity_tf2 / user_command_speed;

  // Build orientation: rotate +X axis to user_command_velocity_normalized
  const tf2::Vector3 x_axis(1.0, 0.0, 0.0);
  double dot = x_axis.dot(user_command_velocity_normalized);
  if (dot > 1.0) dot = 1.0;
  if (dot < -1.0) dot = -1.0;

  tf2::Quaternion q;
  if (dot > 1.0 - 1e-9) {
    // Aligned with same direction: no rotation
    q.setValue(0, 0, 0, 1);
  } else if (dot < -1.0 + 1e-9) {
    // Aligned with opposite direction: rotate 180° around any axis orthogonal to X.
    //   Here, we choose Z axis
    q.setValue(0, 0, 1, 0);
  } else {
    tf2::Vector3 axis = x_axis.cross(user_command_velocity_normalized);
    axis.normalize();
    const double angle = std::acos(dot);
    q.setRotation(axis, angle);
  }

  user_command_velocity_marker_.action = visualization_msgs::Marker::ADD;
  user_command_velocity_marker_.pose.position = desired_gripper_pose_.pose.position;
  user_command_velocity_marker_.pose.orientation.x = q.x();
  user_command_velocity_marker_.pose.orientation.y = q.y();
  user_command_velocity_marker_.pose.orientation.z = q.z();
  user_command_velocity_marker_.pose.orientation.w = q.w();

  // Arrow size: scale.x = shaft length; y/z = diameters; head length auto-scales with y/z
  // We'll split the total length: shaft + head
  const double head_length_ratio = 0.1;  // head length = 10% of total length
  const double head_length = std::max(0.001, head_length_ratio * user_command_speed);
  const double shaft_length = std::max(0.0, user_command_speed - head_length);
  user_command_velocity_marker_.scale.x = shaft_length;
  user_command_velocity_marker_.scale.y = 0.01;  // shaft diameter
  user_command_velocity_marker_.scale.z = 0.01;  // head diameter
  user_command_velocity_marker_.color.r = 0.1f;
  user_command_velocity_marker_.color.g = 0.7f;
  user_command_velocity_marker_.color.b = 1.0f;
  user_command_velocity_marker_.color.a = 1.0f;
}

void MotionControllerTeleoperation::calculateGripperVelocityMarker() {
  gripper_velocity_marker_.header.frame_id = "tm_base";
  gripper_velocity_marker_.header.stamp = ros::Time::now();
  gripper_velocity_marker_.ns = "gripper_velocity";
  gripper_velocity_marker_.id = 0;
  gripper_velocity_marker_.type = visualization_msgs::Marker::ARROW;

  const tf2::Vector3 gripper_velocity_tf2(
      gripper_velocity_.x,
      gripper_velocity_.y,
      gripper_velocity_.z
  );
  const double gripper_speed = gripper_velocity_tf2.length();
  if (gripper_speed < 1e-3) {
    gripper_velocity_marker_.action = visualization_msgs::Marker::DELETE;
    return;
  }
  tf2::Vector3 gripper_velocity_normalized = 
      gripper_velocity_tf2 / gripper_speed;

  // Build orientation: rotate +X axis to gripper_velocity_normalized
  const tf2::Vector3 x_axis(1.0, 0.0, 0.0);
  double dot = x_axis.dot(gripper_velocity_normalized);
  if (dot > 1.0) dot = 1.0;
  if (dot < -1.0) dot = -1.0;

  tf2::Quaternion q;
  if (dot > 1.0 - 1e-9) {
    // Aligned with same direction: no rotation
    q.setValue(0, 0, 0, 1);
  } else if (dot < -1.0 + 1e-9) {
    // Aligned with opposite direction: rotate 180° around any axis orthogonal to X.
    //   Here, we choose Z axis
    q.setValue(0, 0, 1, 0);
  } else {
    tf2::Vector3 axis = x_axis.cross(gripper_velocity_normalized);
    axis.normalize();
    const double angle = std::acos(dot);
    q.setRotation(axis, angle);
  }

  gripper_velocity_marker_.action = visualization_msgs::Marker::ADD;
  gripper_velocity_marker_.pose.position = current_gripper_pose_.pose.position;
  gripper_velocity_marker_.pose.orientation.x = q.x();
  gripper_velocity_marker_.pose.orientation.y = q.y();
  gripper_velocity_marker_.pose.orientation.z = q.z();
  gripper_velocity_marker_.pose.orientation.w = q.w();

  // Arrow size: scale.x = shaft length; y/z = diameters; head length auto-scales with y/z
  // We'll split the total length: shaft + head
  const double head_length_ratio = 0.1;  // head length = 10% of total length
  const double head_length = std::max(0.001, head_length_ratio * gripper_speed);
  const double shaft_length = std::max(0.0, gripper_speed - head_length);
  gripper_velocity_marker_.scale.x = shaft_length;
  gripper_velocity_marker_.scale.y = 0.01;  // shaft diameter
  gripper_velocity_marker_.scale.z = 0.01;  // head diameter
  gripper_velocity_marker_.color.r = 0.5f;
  gripper_velocity_marker_.color.g = 0.0f;
  gripper_velocity_marker_.color.b = 0.5f;
  gripper_velocity_marker_.color.a = 1.0f;
}