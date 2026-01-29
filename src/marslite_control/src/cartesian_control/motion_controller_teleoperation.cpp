#include "cartesian_control/motion_controller_teleoperation.h"
#include <cmath>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

MotionControllerTeleoperation::MotionControllerTeleoperation(const ros::NodeHandle& nh)
    : nh_(nh), loop_rate_(50), 
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
  // TF yaw offset from control view direction
  // [Note] TF is defined as: /tm_base -> /tm_gripper
  // [Note] Actual_yaw = control_view_angle_ + control_view_offset_
  control_view_offset_ = use_sim_ ? 0.0 : M_PI / 2;

  ros::NodeHandle pnh("~");
  pnh.param("use_sim", use_sim_, false);
  pnh.param("use_shared_controller", use_shared_controller_, false);

  nh_.getParam("mobile_platform/linear_scale", platform_linear_scale_);
  nh_.getParam("mobile_platform/angular_scale", platform_angular_scale_);
  nh_.getParam("robotic_arm/position/radius_scale", gripper_position_radius_scale_);
  nh_.getParam("robotic_arm/position/yaw_scale", gripper_position_yaw_scale_);
  nh_.getParam("robotic_arm/position/height_scale", gripper_position_height_scale_);
  nh_.getParam("robotic_arm/orientation/roll_scale", gripper_orientation_roll_scale_);
  nh_.getParam("robotic_arm/orientation/pitch_scale", gripper_orientation_pitch_scale_);
  nh_.getParam("robotic_arm/orientation/yaw_scale", gripper_orientation_yaw_scale_);
  
  ROS_INFO_STREAM(std::boolalpha << "Parameters: " 
      << "\n * use_sim: " << use_sim_
      << "\n * use_shared_controller: " << use_shared_controller_
      << "\n * platform_linear_scale: " << platform_linear_scale_
      << "\n * platform_angular_scale: " << platform_angular_scale_
      << "\n * gripper_position_radius_scale: " << gripper_position_radius_scale_
      << "\n * gripper_position_yaw_scale: " << gripper_position_yaw_scale_
      << "\n * gripper_position_height_scale: " << gripper_position_height_scale_
      << "\n * gripper_orientation_roll_scale: " << gripper_orientation_roll_scale_
      << "\n * gripper_orientation_pitch_scale: " << gripper_orientation_pitch_scale_
      << "\n * gripper_orientation_yaw_scale: " << gripper_orientation_yaw_scale_
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
  initial_left_controller_pose_.header.frame_id = "vr_origin";
  current_left_controller_pose_.header.frame_id = "vr_origin";
  shoulder_position_.header.frame_id = "vr_origin";

  initial_gripper_pose_.header.frame_id = "tm_base";
  current_gripper_pose_.header.frame_id = "tm_base";
  desired_gripper_pose_.header.frame_id = "tm_base";
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
    current_left_controller_pose_ = *msg;
    current_left_controller_pose_.header.stamp = ros::Time::now();
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
          msg->axes[1] * platform_linear_scale_ : 0.0;
    case 1:
      // [0] horizontal axis: Turn left/right
      // [Note] input: left- / right+
      mobile_platform_velocity_.angular.z = std::abs(msg->axes[0]) > kAnalogJoystickDeadzone ?
          - msg->axes[0] * platform_angular_scale_ : 0.0;
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
        shoulder_position_.point = current_left_controller_pose_.pose.position;
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
  const double dx = current_gripper_pose_.pose.position.x - target_pose.pose.position.x;
  const double dy = current_gripper_pose_.pose.position.y - target_pose.pose.position.y;
  const double dz = current_gripper_pose_.pose.position.z - target_pose.pose.position.z;
  const double position_error = sqrt(dx*dx + dy*dy + dz*dz);
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
  initial_gripper_pose_ = desired_gripper_pose_ = current_gripper_pose_;
}

void MotionControllerTeleoperation::initializeLeftControllerPose() {
  initial_left_controller_pose_ = current_left_controller_pose_;
}

void MotionControllerTeleoperation::updateGripperPose() {
  const std::string source_frame = use_sim_ ? "robotiq_85_base_link" : "tm_gripper";
  const std::string target_frame = "tm_base";
  current_gripper_pose_ = tf2_listener_.lookupTransform<geometry_msgs::PoseStamped>(
      target_frame, source_frame
  );
  current_gripper_pose_publisher_.publish(current_gripper_pose_);
}

void MotionControllerTeleoperation::updateGripperVelocity() {
  geometry_msgs::PointStamped waypoint;
  waypoint.header = current_gripper_pose_.header;
  waypoint.point = current_gripper_pose_.pose.position;
  gripper_velocity_estimator_.addWaypoint(waypoint);
  gripper_velocity_estimator_.estimateVelocity();
  gripper_velocity_ = gripper_velocity_estimator_.getEstimatedVelocity();
}

void MotionControllerTeleoperation::calculateDesiredGripperPose() {
  desired_gripper_pose_.header.stamp = ros::Time::now();
  if (is_position_change_enabled_) {
    this->calculateDesiredGripperPosition();
  }
  if (is_orientation_change_enabled_) {
    this->calculateDesiredGripperOrientation();
  }
}

void MotionControllerTeleoperation::calculateDesiredGripperPosition() {
  CylindricalPoint position_difference = this->getCylindricalPositionDifference();
  CylindricalPoint scaled_position_difference = this->scaleCylindricalPositionDifference(position_difference);
  this->applyCylindricalPositionDifference(scaled_position_difference);
}

CylindricalPoint MotionControllerTeleoperation::getCylindricalPositionDifference() {
  const CylindricalPoint initial_cylindrical_position = {
      std::hypot(initial_left_controller_pose_.pose.position.x, initial_left_controller_pose_.pose.position.y),
      std::atan2(initial_left_controller_pose_.pose.position.y, initial_left_controller_pose_.pose.position.x),
      initial_left_controller_pose_.pose.position.z
  };
  const CylindricalPoint current_cylindrical_position = {
      std::hypot(current_left_controller_pose_.pose.position.x, current_left_controller_pose_.pose.position.y),
      std::atan2(current_left_controller_pose_.pose.position.y, current_left_controller_pose_.pose.position.x),
      current_left_controller_pose_.pose.position.z
  };
  const CylindricalPoint position_difference = {
      current_cylindrical_position.radius - initial_cylindrical_position.radius,
      this->restrictAngleWithinPI(
          current_cylindrical_position.yaw - initial_cylindrical_position.yaw
      ),
      current_cylindrical_position.height - initial_cylindrical_position.height
  };
  return position_difference;
}

CylindricalPoint MotionControllerTeleoperation::scaleCylindricalPositionDifference(
    const CylindricalPoint& position_difference) {
  CylindricalPoint scaled_point_difference;
  scaled_point_difference.radius = position_difference.radius * gripper_position_radius_scale_;
  scaled_point_difference.yaw = position_difference.yaw * gripper_position_yaw_scale_;
  scaled_point_difference.height = position_difference.height * gripper_position_height_scale_;
  return scaled_point_difference;
}

void MotionControllerTeleoperation::applyCylindricalPositionDifference(
    const CylindricalPoint& scaled_position_difference) {
  
  Eigen::Vector3d initial_gripper_position(
      initial_gripper_pose_.pose.position.x,
      initial_gripper_pose_.pose.position.y,
      initial_gripper_pose_.pose.position.z
  );
  Eigen::Quaterniond initial_gripper_quaternion(
      initial_gripper_pose_.pose.orientation.w,
      initial_gripper_pose_.pose.orientation.x,
      initial_gripper_pose_.pose.orientation.y,
      initial_gripper_pose_.pose.orientation.z
  );
  Eigen::Matrix3d initial_gripper_rotation_matrix =
      initial_gripper_quaternion.toRotationMatrix();

  const geometry_msgs::TransformStamped T_gripper_wrist =
      tf2_listener_.lookupTransform<geometry_msgs::TransformStamped>(
          "tm_wrist_2_link", "tm_gripper"
      );
  const Eigen::Vector3d p_gripper_wrist(
      T_gripper_wrist.transform.translation.x,
      T_gripper_wrist.transform.translation.y,
      T_gripper_wrist.transform.translation.z
  );
  const Eigen::Vector3d initial_wrist_position = initial_gripper_position -
      (initial_gripper_rotation_matrix * p_gripper_wrist);
  const CylindricalPoint initial_wrist_cylindrical_position = {
      std::hypot(initial_wrist_position.x(), initial_wrist_position.y()),
      std::atan2(initial_wrist_position.y(), initial_wrist_position.x()),
      initial_wrist_position.z()
  };
  CylindricalPoint desired_wrist_cylindrical_position;
  desired_wrist_cylindrical_position.radius =
      initial_wrist_cylindrical_position.radius + scaled_position_difference.radius;
  desired_wrist_cylindrical_position.yaw =
      initial_wrist_cylindrical_position.yaw + scaled_position_difference.yaw;
  desired_wrist_cylindrical_position.height =
      initial_wrist_cylindrical_position.height + scaled_position_difference.height;
  
  Eigen::Vector3d desired_wrist_position;
  desired_wrist_position.x() = desired_wrist_cylindrical_position.radius *
      std::cos(desired_wrist_cylindrical_position.yaw);
  desired_wrist_position.y() = desired_wrist_cylindrical_position.radius *
      std::sin(desired_wrist_cylindrical_position.yaw);
  desired_wrist_position.z() = desired_wrist_cylindrical_position.height;
  
  Eigen::AngleAxisd yaw_rotation(scaled_position_difference.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond desired_gripper_quaternion = yaw_rotation * initial_gripper_quaternion;
  Eigen::Matrix3d desired_rotation_matrix = desired_gripper_quaternion.toRotationMatrix();
  Eigen::Vector3d desired_gripper_position = desired_wrist_position +
      (desired_rotation_matrix * p_gripper_wrist);
    
  desired_gripper_pose_.pose.position.x = desired_gripper_position.x();
  desired_gripper_pose_.pose.position.y = desired_gripper_position.y();
  desired_gripper_pose_.pose.position.z = desired_gripper_position.z();
  desired_gripper_pose_.pose.orientation.x = desired_gripper_quaternion.x();
  desired_gripper_pose_.pose.orientation.y= desired_gripper_quaternion.y();
  desired_gripper_pose_.pose.orientation.z = desired_gripper_quaternion.z();
  desired_gripper_pose_.pose.orientation.w = desired_gripper_quaternion.w();
}

void MotionControllerTeleoperation::calculateDesiredGripperOrientation() {
  const RPY orientation_difference = this->getOrientationDifference();
  const RPY scaled_orientation_difference = this->scaleOrientationDifference(orientation_difference);
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
  scaled_orientation_difference.roll = orientation_difference.roll * gripper_orientation_roll_scale_;
  scaled_orientation_difference.pitch = orientation_difference.pitch * gripper_orientation_pitch_scale_;
  scaled_orientation_difference.yaw = orientation_difference.yaw * gripper_orientation_yaw_scale_;
  return scaled_orientation_difference;
}

void MotionControllerTeleoperation::applyOrientationDifference(const RPY& scaled_orientation_difference) {
  Eigen::Vector3d initial_gripper_position(
      initial_gripper_pose_.pose.position.x,
      initial_gripper_pose_.pose.position.y,
      initial_gripper_pose_.pose.position.z
  );
  Eigen::Quaterniond initial_gripper_quaternion(
      initial_gripper_pose_.pose.orientation.w,
      initial_gripper_pose_.pose.orientation.x, 
      initial_gripper_pose_.pose.orientation.y, 
      initial_gripper_pose_.pose.orientation.z
  );
  Eigen::Matrix3d initial_gripper_rotation_matrix =
      initial_gripper_quaternion.toRotationMatrix();

  const geometry_msgs::TransformStamped T_gripper_wrist =
      tf2_listener_.lookupTransform<geometry_msgs::TransformStamped>(
          "tm_wrist_2_link", "tm_gripper"
      );
  const Eigen::Vector3d p_gripper_wrist(
      T_gripper_wrist.transform.translation.x,
      T_gripper_wrist.transform.translation.y,
      T_gripper_wrist.transform.translation.z
  );
  const Eigen::Vector3d wrist_position = initial_gripper_position -
      (initial_gripper_rotation_matrix * p_gripper_wrist);

  Eigen::AngleAxisd yaw_rotation(scaled_orientation_difference.yaw, Eigen::Vector3d::UnitZ());  // global
  Eigen::AngleAxisd pitch_rotation(scaled_orientation_difference.pitch, Eigen::Vector3d::UnitX());  // local
  Eigen::AngleAxisd roll_rotation(-scaled_orientation_difference.roll, Eigen::Vector3d::UnitZ());  // local
  // = global_rotation * initial_quat * local_rotation
  Eigen::Quaterniond desired_gripper_quaternion = yaw_rotation * initial_gripper_quaternion * pitch_rotation * roll_rotation;
  Eigen::Matrix3d desired_rotation_matrix = desired_gripper_quaternion.toRotationMatrix();
  Eigen::Vector3d desired_gripper_position = wrist_position +
      (desired_rotation_matrix * p_gripper_wrist);

  desired_gripper_pose_.pose.position.x = desired_gripper_position.x();
  desired_gripper_pose_.pose.position.y = desired_gripper_position.y();
  desired_gripper_pose_.pose.position.z = desired_gripper_position.z();
  desired_gripper_pose_.pose.orientation.x = desired_gripper_quaternion.x();
  desired_gripper_pose_.pose.orientation.y = desired_gripper_quaternion.y();
  desired_gripper_pose_.pose.orientation.z = desired_gripper_quaternion.z();
  desired_gripper_pose_.pose.orientation.w = desired_gripper_quaternion.w();
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
  gripper_velocity_marker_.lifetime = ros::Duration(0.2);
}