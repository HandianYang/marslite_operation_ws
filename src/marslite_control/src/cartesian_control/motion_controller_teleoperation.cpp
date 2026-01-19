#include "cartesian_control/motion_controller_teleoperation.h"
#include <cmath>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

MotionControllerTeleoperation::MotionControllerTeleoperation(const ros::NodeHandle& nh)
    : nh_(nh), loop_rate_(50), initial_lateral_offset_(0.0),
      is_position_change_enabled_(false),
      is_orientation_change_enabled_(false),
      is_shoulder_position_calibrated_(false) {
  this->parseParameters();
  this->initializePublishers();
  this->initializeSubscribers();
  this->initializeHeadersForMessages();
  this->initializeVelocityEstimators();
  reset_service_ = nh_.advertiseService("reset_teleop_origin",
      &MotionControllerTeleoperation::resetCallback, this
  );
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void MotionControllerTeleoperation::teleoperateToPose(const geometry_msgs::PoseStamped& target_pose) {
  this->initializeGripperPose();
  target_frame_publisher_.publish(target_pose);
  
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
    this->updateGripperPose();
    loop_rate_.sleep();
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Reached the target pose. Remember to calibrate your shoulder "
      << "position before teleoperation.");
}

geometry_msgs::PoseStamped MotionControllerTeleoperation::generateInitialGripperPose(
    const double& control_view_angle,
    const double& control_view_offset,
    const bool& use_sim) {
  // Pre-defined gripper pose parameters:
  const double kGripperPoseRadius  = 0.538;  // [m]
  const double kGripperPoseYOffset = -0.122; // [m]
  const double kGripperPoseHeight  = 0.666;  // [m]
  const double kGripperPoseRoll  = use_sim ? 0.0 : M_PI / 2; // [rad]
  const double kGripperPosePitch = 0.000; // [rad]
  const double kGripperPoseYaw   = control_view_angle + control_view_offset; // [rad]

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


void MotionControllerTeleoperation::run() {
  this->initializeGripperPose();
  this->initializeLeftControllerPose();
  while (nh_.ok()) {
    this->updateGripperPose();
    {
      std::lock_guard<std::mutex> lock(desired_gripper_pose_mutex_);
      if (this->isAnySafetyButtonPressed()) {
        if (is_shoulder_position_calibrated_) {
          this->calculateDesiredGripperPose();
        } else {
          ROS_WARN_STREAM_THROTTLE(5, "Shoulder position is not calibrated yet. "
            "Please place the left controller on your left shoulder and "
            "press the X button to calibrate.");
        }

      } else {
        this->initializeGripperPose();
        this->initializeLeftControllerPose();
      }

      this->updateGripperVelocity();
      this->calculateGripperVelocityMarker();
      this->calculateUserCommandVelocity();
      this->calculateUserCommandVelocityMarker();

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
  pnh.param("linear_platform_velocity_scale", linear_platform_velocity_scale_, 0.15);
  pnh.param("angular_platform_velocity_scale", angular_platform_velocity_scale_, 0.15);
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
      << "\n * linear_platform_velocity_scale: " << linear_platform_velocity_scale_
      << "\n * angular_platform_velocity_scale: " << angular_platform_velocity_scale_
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
  const std::string desired_gripper_pose_topic = use_shared_controller_ ?
      "/marslite_control/user_desired_gripper_pose" :
      "/target_frame";
  const std::string gripper_status_topic = use_shared_controller_ ?
      "/marslite_control/user_desired_gripper_status" :
      "/gripper/cmd_gripper";
  
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      desired_gripper_pose_topic, 1);
  current_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/marslite_control/gripper_pose", 1);
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
      "/vr_controller/pose", 1,
      &MotionControllerTeleoperation::leftControllerPoseCallback, this
  );
  left_controller_joy_subscriber_ = nh_.subscribe(
      "/vr_controller/joy", 1,
      &MotionControllerTeleoperation::leftControllerJoyCallback, this
  );
}

void MotionControllerTeleoperation::initializeHeadersForMessages() {
  initial_left_controller_hybrid_pose_.header.frame_id = "vr_origin";
  current_left_controller_hybrid_pose_.header.frame_id = "vr_origin";
  shoulder_position_.header.frame_id = "vr_origin";

  initial_gripper_hybrid_pose_.header.frame_id = "tm_base";
  current_gripper_hybrid_pose_.header.frame_id = "tm_base";
  desired_gripper_hybrid_pose_.header.frame_id = "tm_base";
  user_command_velocity_marker_.header.frame_id = "tm_base";
  gripper_velocity_marker_.header.frame_id = "tm_base";
}


void MotionControllerTeleoperation::initializeVelocityEstimators() {
  user_command_velocity_estimator_.setBufferSize(kEstimatorBufferSize);
  user_command_velocity_estimator_.setMinSpeed(kEstimatorMinSpeed);
  gripper_velocity_estimator_.setBufferSize(kEstimatorBufferSize);
  gripper_velocity_estimator_.setMinSpeed(kEstimatorMinSpeed);
}


void MotionControllerTeleoperation::leftControllerPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (is_shoulder_position_calibrated_) {
    current_left_controller_hybrid_pose_.updateFromCartesianPose(*msg, control_view_offset_);
    current_left_controller_hybrid_pose_.header.stamp = ros::Time::now();
  }
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
      // [1] vertical axis: Move forward/backward
      // [Note] input: forward+ / backward-
      mobile_platform_velocity_.linear.x = std::abs(msg->axes[1]) > kAnalogJoystickDeadzone ?
          msg->axes[1] * linear_platform_velocity_scale_ : 0.0;
    case 1:
      // [0] horizontal axis: Turn left/right
      // [Note] input: left- / right+
      mobile_platform_velocity_.angular.z = std::abs(msg->axes[0]) > kAnalogJoystickDeadzone ?
          - msg->axes[0] * angular_platform_velocity_scale_ : 0.0;
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick axes (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }

  switch (msg->buttons.size()) {
    case 3:
      // [2] stick button: Send `record_signal` to `shared_control`
      if (msg->buttons[2] == 1) {
        record_signal_.data = true;
        record_signal_publisher_.publish(record_signal_);
      }
    case 2:
      // [1] Y button: Close/Open the gripper
      if (msg->buttons[1] == 1) {
        this->toggleGripperStatus();
      }
    case 1:
      // [0] X button: Calibrate the shoulder point
      if (msg->buttons[0] == 1) {
        shoulder_position_.point = current_left_controller_hybrid_pose_.cartesian_position;
        is_shoulder_position_calibrated_ = true;
        ROS_INFO_STREAM("Complete calibration of the shoulder point at: ("
            << shoulder_position_.point.x << ", "
            << shoulder_position_.point.y << ", "
            << shoulder_position_.point.z << ")"
        );
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

bool MotionControllerTeleoperation::resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  {
    std::lock_guard<std::mutex> lock(desired_gripper_pose_mutex_);
    this->updateGripperPose();
    this->initializeGripperPose();
    this->initializeLeftControllerPose();
    
    res.success = true;
    res.message = "Origins reset.";
    return true;
  }
}

const bool MotionControllerTeleoperation::targetPoseIsReached(
    const geometry_msgs::PoseStamped& target_pose) const {
  const double dx = current_gripper_hybrid_pose_.cartesian_position.x - target_pose.pose.position.x;
  const double dy = current_gripper_hybrid_pose_.cartesian_position.y - target_pose.pose.position.y;
  const double dz = current_gripper_hybrid_pose_.cartesian_position.z - target_pose.pose.position.z;
  const double position_error = sqrt(dx*dx + dy*dy + dz*dz);
  if (position_error > kPositionTolerance)  return false;

  auto toEig = [](const geometry_msgs::Quaternion& q){
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  };
  Eigen::Quaterniond q1 = toEig(current_gripper_hybrid_pose_.orientation);
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
  initial_gripper_hybrid_pose_ = desired_gripper_hybrid_pose_
                               = current_gripper_hybrid_pose_;
  
  // Update lateral offset
  const double control_view_angle = current_gripper_hybrid_pose_.cylindrical_position.yaw - control_view_offset_;
  initial_lateral_offset_ = 
      - current_gripper_hybrid_pose_.cartesian_position.x * std::sin(control_view_angle) +
      current_gripper_hybrid_pose_.cartesian_position.y * std::cos(control_view_angle);
}

void MotionControllerTeleoperation::initializeLeftControllerPose() {
  initial_left_controller_hybrid_pose_ = current_left_controller_hybrid_pose_;
  previous_left_controller_hybrid_pose_ = current_left_controller_hybrid_pose_;
  accumulated_radius_difference_ = 0.0;
}

void MotionControllerTeleoperation::updateGripperPose() {
  const std::string source_frame = use_sim_ ? "robotiq_85_base_link" : "tm_gripper";
  current_gripper_hybrid_pose_.updateFromCartesianPose(
      tf2_listener_.lookupTransform<geometry_msgs::PoseStamped>(
          "tm_base", source_frame
      ),
      control_view_offset_
  );

  geometry_msgs::PoseStamped current_gripper_pose_msg
      = current_gripper_hybrid_pose_.toCartesianPoseStamped();
  current_gripper_pose_publisher_.publish(current_gripper_pose_msg);
}

void MotionControllerTeleoperation::updateGripperVelocity() {
  geometry_msgs::PointStamped waypoint;
  waypoint.header = current_gripper_hybrid_pose_.header;
  waypoint.point = current_gripper_hybrid_pose_.cartesian_position;
  gripper_velocity_estimator_.addWaypoint(waypoint);
  gripper_velocity_estimator_.estimateVelocity();
  gripper_velocity_ = gripper_velocity_estimator_.getEstimatedVelocity();
}

void MotionControllerTeleoperation::calculateDesiredGripperPose() {
  desired_gripper_hybrid_pose_.header.stamp = ros::Time::now();
  if (is_orientation_change_enabled_) {
    this->calculateDesiredGripperOrientation();
  }

  if (is_position_change_enabled_) {
    this->calculateDesiredGripperPosition();
  }
}

void MotionControllerTeleoperation::calculateDesiredGripperOrientation() {
  const RPY orientation_difference = this->getOrientationDifference();
  const RPY scaled_orientation_difference = this->scaleOrientationDifference(orientation_difference);
  this->applyOrientationDifference(scaled_orientation_difference);
}

RPY MotionControllerTeleoperation::getOrientationDifference() {
  RPY initial_left_controller_rpy;
  initial_left_controller_rpy.convertFromPoseStamped(
      initial_left_controller_hybrid_pose_.toCartesianPoseStamped()
  );
  RPY current_left_controller_rpy;
  current_left_controller_rpy.convertFromPoseStamped(
      current_left_controller_hybrid_pose_.toCartesianPoseStamped()
  );

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
  Eigen::Vector3d initial_gripper_position(
      initial_gripper_hybrid_pose_.cartesian_position.x, 
      initial_gripper_hybrid_pose_.cartesian_position.y, 
      initial_gripper_hybrid_pose_.cartesian_position.z
  );
  Eigen::Quaterniond initial_gripper_quaternion(
      initial_gripper_hybrid_pose_.orientation.w,
      initial_gripper_hybrid_pose_.orientation.x, 
      initial_gripper_hybrid_pose_.orientation.y, 
      initial_gripper_hybrid_pose_.orientation.z
  );
  Eigen::Matrix3d initial_gripper_rotation_matrix =
      initial_gripper_quaternion.toRotationMatrix();

  const geometry_msgs::TransformStamped wrist_2_to_gripper_tf =
      tf2_listener_.lookupTransform<geometry_msgs::TransformStamped>(
          "tm_wrist_2_link" , "tm_gripper" 
      );
  const Eigen::Vector3d wrist_to_gripper_vector(
      wrist_2_to_gripper_tf.transform.translation.x,
      wrist_2_to_gripper_tf.transform.translation.y,
      wrist_2_to_gripper_tf.transform.translation.z
  );
  const Eigen::Vector3d wrist_position = initial_gripper_position -
      (initial_gripper_rotation_matrix * wrist_to_gripper_vector);
  
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

  Eigen::AngleAxisd yaw_rotation(transformed_orientation_difference.yaw, Eigen::Vector3d::UnitZ());  // global
  Eigen::AngleAxisd pitch_rotation(transformed_orientation_difference.pitch, Eigen::Vector3d::UnitZ());  // local
  Eigen::AngleAxisd roll_rotation(transformed_orientation_difference.roll, Eigen::Vector3d::UnitX());  // local
  // = global_rotation * initial_quat * local_rotation
  Eigen::Quaterniond desired_gripper_quaternion = yaw_rotation * initial_gripper_quaternion * pitch_rotation * roll_rotation;
  Eigen::Matrix3d desired_rotation_matrix = desired_gripper_quaternion.toRotationMatrix();
  Eigen::Vector3d desired_gripper_position = wrist_position +
      (desired_rotation_matrix * wrist_to_gripper_vector);

  desired_gripper_hybrid_pose_.cartesian_position.x = desired_gripper_position.x();
  desired_gripper_hybrid_pose_.cartesian_position.y = desired_gripper_position.y();
  desired_gripper_hybrid_pose_.cartesian_position.z = desired_gripper_position.z();
  desired_gripper_hybrid_pose_.orientation.x = desired_gripper_quaternion.x();
  desired_gripper_hybrid_pose_.orientation.y = desired_gripper_quaternion.y();
  desired_gripper_hybrid_pose_.orientation.z = desired_gripper_quaternion.z();
  desired_gripper_hybrid_pose_.orientation.w = desired_gripper_quaternion.w();
}

void MotionControllerTeleoperation::calculateDesiredGripperPosition() {
  CylindricalPoint position_difference = this->getCylindricalPositionDifference();
  CylindricalPoint scaled_position_difference = this->scaleCylindricalPositionDifference(position_difference);
  this->applyCylindricalPositionDifference(scaled_position_difference);
}

CylindricalPoint MotionControllerTeleoperation::getCylindricalPositionDifference() {
  // Since the radius cannot be obtained directly from cylindrical coordinates,
  //   we calculate the radius difference by comparing the previous and current
  //   left controller positions in Cartesian coordinates.
  const double previous_hand_radius = std::hypot(
      previous_left_controller_hybrid_pose_.cartesian_position.x,
      previous_left_controller_hybrid_pose_.cartesian_position.y
  );
  const double current_hand_radius = std::hypot(
      current_left_controller_hybrid_pose_.cartesian_position.x,
      current_left_controller_hybrid_pose_.cartesian_position.y
  );
  const double step_radius = current_hand_radius - previous_hand_radius;
  accumulated_radius_difference_ += step_radius;
  previous_left_controller_hybrid_pose_ = current_left_controller_hybrid_pose_;

  // Calculate full cylindrical position difference
  CylindricalPoint position_difference;
  position_difference.radius = accumulated_radius_difference_;
  position_difference.yaw = this->restrictAngleWithinPI(
      current_left_controller_hybrid_pose_.cylindrical_position.yaw -
      initial_left_controller_hybrid_pose_.cylindrical_position.yaw);
  position_difference.radius = accumulated_radius_difference_;
  position_difference.height =
      current_left_controller_hybrid_pose_.cylindrical_position.height -
      initial_left_controller_hybrid_pose_.cylindrical_position.height;
  return position_difference;
}

CylindricalPoint MotionControllerTeleoperation::scaleCylindricalPositionDifference(
    const CylindricalPoint& position_difference) {
  CylindricalPoint scaled_point_difference;
  scaled_point_difference.radius = position_difference.radius * position_scale_;
  scaled_point_difference.yaw = position_difference.yaw * orientation_scale_;
  scaled_point_difference.height = position_difference.height * position_scale_;
  return scaled_point_difference;
}

void MotionControllerTeleoperation::applyCylindricalPositionDifference(
    const CylindricalPoint& scaled_position_difference) {
  CylindricalPoint desired_gripper_cylindrical_position;
  desired_gripper_cylindrical_position.radius =
      initial_gripper_hybrid_pose_.cylindrical_position.radius + scaled_position_difference.radius;
  desired_gripper_cylindrical_position.yaw =
      initial_gripper_hybrid_pose_.cylindrical_position.yaw + scaled_position_difference.yaw;
  desired_gripper_cylindrical_position.height =
      initial_gripper_hybrid_pose_.cylindrical_position.height + scaled_position_difference.height;

  desired_gripper_hybrid_pose_.updateFromCylindricalPoint(
      desired_gripper_cylindrical_position,
      control_view_offset_,
      initial_lateral_offset_
  );
}

void MotionControllerTeleoperation::calculateUserCommandVelocity() {
  geometry_msgs::PointStamped waypoint;
  waypoint.header = desired_gripper_hybrid_pose_.header;
  waypoint.point = desired_gripper_hybrid_pose_.cartesian_position;
  user_command_velocity_estimator_.addWaypoint(waypoint);
  user_command_velocity_estimator_.estimateVelocity();
  user_command_velocity_ = user_command_velocity_estimator_.getEstimatedVelocity();
}

void MotionControllerTeleoperation::calculateUserCommandVelocityMarker() {
  const tf2::Vector3 user_command_velocity_tf2(
      user_command_velocity_.x,
      user_command_velocity_.y,
      user_command_velocity_.z
  );
  const double user_command_speed = user_command_velocity_tf2.length();
  if (user_command_speed < 1e-3) {
    return;
  }

  user_command_velocity_marker_.header.stamp = ros::Time::now();
  user_command_velocity_marker_.ns = "user_command_velocity";
  user_command_velocity_marker_.id = 0;
  user_command_velocity_marker_.type = visualization_msgs::Marker::ARROW;

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
  user_command_velocity_marker_.pose.position = desired_gripper_hybrid_pose_.cartesian_position;
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
  user_command_velocity_marker_.lifetime = ros::Duration(0.2);
}

void MotionControllerTeleoperation::calculateGripperVelocityMarker() {
  const tf2::Vector3 gripper_velocity_tf2(
      gripper_velocity_.x,
      gripper_velocity_.y,
      gripper_velocity_.z
  );
  const double gripper_speed = gripper_velocity_tf2.length();
  if (gripper_speed < 1e-3) {
    return;
  }

  gripper_velocity_marker_.header.stamp = ros::Time::now();
  gripper_velocity_marker_.ns = "gripper_velocity";
  gripper_velocity_marker_.id = 0;
  gripper_velocity_marker_.type = visualization_msgs::Marker::ARROW;

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
  gripper_velocity_marker_.pose.position = current_gripper_hybrid_pose_.cartesian_position;
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
  gripper_velocity_marker_.lifetime = ros::Duration(0.2);
}