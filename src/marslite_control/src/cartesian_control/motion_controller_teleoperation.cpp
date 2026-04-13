#include "cartesian_control/motion_controller_teleoperation.h"
#include <cmath>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

MotionControllerTeleoperation::MotionControllerTeleoperation(const ros::NodeHandle& nh)
    : nh_(nh), loop_rate_(50),
      is_ready_to_teleop_(false),
      is_position_change_enabled_(false),
      is_orientation_change_enabled_(false),
      is_shoulder_position_calibrated_(false),
      control_view_angle_(0.0),
      gripper_position_radius_min_(0.10),
      gripper_position_radius_max_(0.65),
      gripper_position_height_min_(0.05),
      gripper_position_height_max_(0.90),
      gripper_orientation_roll_limit_(M_PI / 4),
      gripper_orientation_pitch_limit_(M_PI / 4),
      gripper_orientation_yaw_limit_(M_PI / 4),
      is_last_published_pose_initialized_(false) {
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

void MotionControllerTeleoperation::resetToReadyPose() {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "tm_base";
  pose.pose.position.x = 0.122;
  pose.pose.position.y = 0.35;
  pose.pose.position.z = 0.6;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707;
  pose.pose.orientation.z = 0.707;
  pose.pose.orientation.w = 0.0;
  this->teleoperateToPose(pose);

  // NOTE: This must be set after teleoperateToPose()
  control_view_angle_ = M_PI / 2;
}

void MotionControllerTeleoperation::resetToFrontPose() {
  control_view_offset_ = use_sim_ ? 0.0 : M_PI / 2;
  this->teleoperateToPose(
      MotionControllerTeleoperation::generateInitialGripperPose(
          0.0, control_view_offset_, use_sim_
      )
  );

  // NOTE: This must be set after teleoperateToPose()
  control_view_angle_ = 0.0;
}

void MotionControllerTeleoperation::resetToLeftPose() {
  control_view_offset_ = use_sim_ ? 0.0 : M_PI / 2;
  this->teleoperateToPose(
      MotionControllerTeleoperation::generateInitialGripperPose(
          M_PI / 2, control_view_offset_, use_sim_
      )
  );

  // NOTE: This must be set after teleoperateToPose()
  control_view_angle_ = M_PI / 2;
}

void MotionControllerTeleoperation::resetToRightPose() {
  control_view_offset_ = use_sim_ ? 0.0 : M_PI / 2;
  this->teleoperateToPose(
      MotionControllerTeleoperation::generateInitialGripperPose(
          -M_PI / 2, control_view_offset_, use_sim_
      )
  );

  // NOTE: This must be set after teleoperateToPose()
  control_view_angle_ = -M_PI / 2;
}


void MotionControllerTeleoperation::teleoperateToPose(const geometry_msgs::PoseStamped& target_pose) {
  is_ready_to_teleop_ = false; // Disable teleoperation during guiding phase

  this->initializeGripperPose();
  // target_frame_publisher_.publish(target_pose);
  desired_gripper_pose_publisher_.publish(target_pose);
  
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
  initial_gripper_pose_ = current_gripper_pose_;

  ROS_INFO_STREAM("Reached the target pose.");
  ROS_INFO_STREAM_ONCE("Remember to calibrate your shoulder position before teleoperation.");
  is_ready_to_teleop_ = true;
}


void MotionControllerTeleoperation::run() {
  this->initializeGripperPose();
  this->initializeLeftControllerPose();
  while (nh_.ok()) {
    this->updateGripperPose();
    if (!is_ready_to_teleop_) {
      loop_rate_.sleep();
      ros::spinOnce();
      continue;
    }

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
      this->printCylindricalPose();
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
  nh_.getParam("robotic_arm/position/radius_min", gripper_position_radius_min_);
  nh_.getParam("robotic_arm/position/radius_max", gripper_position_radius_max_);
  nh_.getParam("robotic_arm/position/height_min", gripper_position_height_min_);
  nh_.getParam("robotic_arm/position/height_max", gripper_position_height_max_);
  nh_.getParam("robotic_arm/orientation/roll_scale", gripper_orientation_roll_scale_);
  nh_.getParam("robotic_arm/orientation/pitch_scale", gripper_orientation_pitch_scale_);
  nh_.getParam("robotic_arm/orientation/yaw_scale", gripper_orientation_yaw_scale_);

  // Orientation limits are specified in degrees in YAML for readability
  double roll_limit_deg = gripper_orientation_roll_limit_ * 180.0 / M_PI;
  double pitch_limit_deg = gripper_orientation_pitch_limit_ * 180.0 / M_PI;
  double yaw_limit_deg = gripper_orientation_yaw_limit_ * 180.0 / M_PI;
  nh_.getParam("robotic_arm/orientation/roll_limit_deg", roll_limit_deg);
  nh_.getParam("robotic_arm/orientation/pitch_limit_deg", pitch_limit_deg);
  nh_.getParam("robotic_arm/orientation/yaw_limit_deg", yaw_limit_deg);
  gripper_orientation_roll_limit_  = roll_limit_deg  * M_PI / 180.0;
  gripper_orientation_pitch_limit_ = pitch_limit_deg * M_PI / 180.0;
  gripper_orientation_yaw_limit_   = yaw_limit_deg   * M_PI / 180.0;

  ROS_INFO_STREAM(std::boolalpha << "Parameters: "
      << "\n * use_sim: " << use_sim_
      << "\n * use_shared_controller: " << use_shared_controller_
      << "\n * platform_linear_scale: " << platform_linear_scale_
      << "\n * platform_angular_scale: " << platform_angular_scale_
      << "\n * gripper_position_radius_scale: " << gripper_position_radius_scale_
      << "\n * gripper_position_yaw_scale: " << gripper_position_yaw_scale_
      << "\n * gripper_position_height_scale: " << gripper_position_height_scale_
      << "\n * gripper_position_radius_min: " << gripper_position_radius_min_
      << "\n * gripper_position_radius_max: " << gripper_position_radius_max_
      << "\n * gripper_position_height_min: " << gripper_position_height_min_
      << "\n * gripper_position_height_max: " << gripper_position_height_max_
      << "\n * gripper_orientation_roll_scale: " << gripper_orientation_roll_scale_
      << "\n * gripper_orientation_pitch_scale: " << gripper_orientation_pitch_scale_
      << "\n * gripper_orientation_yaw_scale: " << gripper_orientation_yaw_scale_
      << "\n * gripper_orientation_roll_limit: " << roll_limit_deg << " deg"
      << "\n * gripper_orientation_pitch_limit: " << pitch_limit_deg << " deg"
      << "\n * gripper_orientation_yaw_limit: " << yaw_limit_deg << " deg"
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
  restart_attempt_signal_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/marslite_control/restart_attempt_signal", 1);
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
  if (!is_ready_to_teleop_) {
    return; // Ignore joystick input if not ready to teleop
  }
  
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
    case 5:
      // [4] B button: Drive the robot back to the initial pose
      if (msg->buttons[4] == 1) {
        // Reset to initial pose after pressing the button for 1 second
        static ros::Time b_button_press_start_time = ros::Time(0);
        if (b_button_press_start_time.isZero()) {
          b_button_press_start_time = ros::Time::now();
        } else if ((ros::Time::now() - b_button_press_start_time).toSec() > 1.0) {
          ROS_INFO("Reset robot pose to the inital pose...");
          b_button_press_start_time = ros::Time(0); // Reset after publishing
          // TODO: consider resetting to different initial poses
          this->resetToFrontPose();
        }
      }
    case 4:
      // [3] A button: Restart this attempt 
      if (msg->buttons[3] == 1) {
        // Send attempt restart signal and reset to initial pose after pressing
        //  the button for 1 second
        static ros::Time a_button_press_start_time = ros::Time(0);
        if (a_button_press_start_time.isZero()) {
          a_button_press_start_time = ros::Time::now();
        } else if ((ros::Time::now() - a_button_press_start_time).toSec() > 1.0) {
          ROS_INFO("Restart this attempt. Reset robot pose to the inital pose...");
          a_button_press_start_time = ros::Time(0); // Reset after publishing
          // TODO: consider resetting to different initial poses
          this->resetToReadyPose();

          std_msgs::Bool signal;
          signal.data = true;
          restart_attempt_signal_publisher_.publish(signal);
        }
      }
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
        shoulder_position_ = current_left_controller_pose_.pose.position;
        is_shoulder_position_calibrated_ = true;
        ROS_INFO_STREAM("Complete calibration of the shoulder point at: ("
            << shoulder_position_.x << ", "
            << shoulder_position_.y << ", "
            << shoulder_position_.z << ")"
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
  // Obtain the actual yaw from the last position of the previous control
  //  session.
  const Eigen::Quaterniond q_init(
      initial_gripper_pose_.pose.orientation.w,
      initial_gripper_pose_.pose.orientation.x,
      initial_gripper_pose_.pose.orientation.y,
      initial_gripper_pose_.pose.orientation.z
  );
  const Eigen::Quaterniond q_curr(
      current_gripper_pose_.pose.orientation.w,
      current_gripper_pose_.pose.orientation.x,
      current_gripper_pose_.pose.orientation.y,
      current_gripper_pose_.pose.orientation.z
  );
  const Eigen::Matrix3d R_delta =
      (q_curr * q_init.inverse()).toRotationMatrix();
  const double achieved_yaw = std::atan2(R_delta(1, 0), R_delta(0, 0));
  // Accumulate the achieved yaw into control_view_angle_
  control_view_angle_ = restrictAngleWithinPI(control_view_angle_ + achieved_yaw);
  
  initial_gripper_pose_ = desired_gripper_pose_
      = last_published_pose_ = current_gripper_pose_;
}

void MotionControllerTeleoperation::initializeLeftControllerPose() {
  initial_left_controller_pose_ = current_left_controller_pose_;
}

void MotionControllerTeleoperation::printCylindricalPose() const {
  const double gx = current_gripper_pose_.pose.position.x;
  const double gy = current_gripper_pose_.pose.position.y;
  const double gz = current_gripper_pose_.pose.position.z;

  const Eigen::Vector2d gripper_xy(gx, gy);
  const Eigen::Vector2d view_xy(
      std::cos(control_view_angle_),
      std::sin(control_view_angle_)
  );
  const double radius = gripper_xy.dot(view_xy);
  const double view_deg = control_view_angle_ * 180.0 / M_PI;

  ROS_INFO_STREAM_THROTTLE(1,
      " radius=" << std::fixed << std::setprecision(3) << radius <<
      " yaw=" << std::setprecision(1) << view_deg << "deg" <<
      " height=" << std::setprecision(3) << gz
  );
}

void MotionControllerTeleoperation::publishDesiredGripperPose() {
  // Rate-limit and quaternion-sign-protect the pose before it reaches the
  // fzi cartesian_motion_controller. Without this, session-boundary
  // discontinuities excite the solver's internal velocity integrator (which
  // only decays at 0.9x / step) and cause persistent drift or wrist rotation.

  if (!is_last_published_pose_initialized_) {
    last_published_pose_ = desired_gripper_pose_;
    is_last_published_pose_initialized_ = true;
    desired_gripper_pose_publisher_.publish(last_published_pose_);
    return;
  }

  // --- Quaternion continuity ---
  // Ensure desired quaternion is in the same hemisphere as the last published
  // one. q and -q represent the same orientation, but the fzi PD controller
  // computes error as R_target * R_current^-1, and KDL's axis-angle extraction
  // can produce a near-pi rotation for a sign flip, exciting wrist joints.
  Eigen::Quaterniond q_desired(
      desired_gripper_pose_.pose.orientation.w,
      desired_gripper_pose_.pose.orientation.x,
      desired_gripper_pose_.pose.orientation.y,
      desired_gripper_pose_.pose.orientation.z
  );
  const Eigen::Quaterniond q_prev(
      last_published_pose_.pose.orientation.w,
      last_published_pose_.pose.orientation.x,
      last_published_pose_.pose.orientation.y,
      last_published_pose_.pose.orientation.z
  );
  if (q_desired.dot(q_prev) < 0.0) {
    q_desired.coeffs() = -q_desired.coeffs();
  }

  // --- Exponential smoothing ---
  // published = prev + alpha * (desired - prev).
  // Unlike a hard rate-limit (constant-speed crawl on stop), this gives
  // exponential convergence: fast at first, then tapering — a natural
  // deceleration feel. During normal operation, the proportional lag is
  // (1-alpha) * per-tick-delta, which is negligible at alpha=0.7.
  const Eigen::Vector3d p_desired(
      desired_gripper_pose_.pose.position.x,
      desired_gripper_pose_.pose.position.y,
      desired_gripper_pose_.pose.position.z
  );
  const Eigen::Vector3d p_prev(
      last_published_pose_.pose.position.x,
      last_published_pose_.pose.position.y,
      last_published_pose_.pose.position.z
  );
  Eigen::Vector3d p_smoothed =
      p_prev + kPoseSmoothingAlpha * (p_desired - p_prev);

  Eigen::Quaterniond q_smoothed =
      q_prev.slerp(kPoseSmoothingAlpha, q_desired);
  q_smoothed.normalize();

  // --- Hard safety cap (belt-and-suspenders) ---
  // Generous enough that normal teleop never triggers it. Only clips genuine
  // runaways where the fzi solver's pre-loaded velocity causes the smoothed
  // output to still move too fast (e.g. residual internal velocity adds to
  // the new command in the same direction).
  Eigen::Vector3d dp = p_smoothed - p_prev;
  const double dp_norm = dp.norm();
  if (dp_norm > kMaxPositionStepPerTick) {
    dp *= kMaxPositionStepPerTick / dp_norm;
    p_smoothed = p_prev + dp;
  }

  const double q_dot_abs = std::abs(q_smoothed.dot(q_prev));
  const double q_angle = 2.0 * std::acos(std::min(1.0, q_dot_abs));
  Eigen::Quaterniond q_publish = q_smoothed;
  if (q_angle > kMaxOrientationStepPerTick && q_angle > 1e-9) {
    const double t = kMaxOrientationStepPerTick / q_angle;
    q_publish = q_prev.slerp(t, q_smoothed);
    q_publish.normalize();
  }
  const Eigen::Vector3d p_publish = p_smoothed;

  // --- Build and publish ---
  last_published_pose_.header = desired_gripper_pose_.header;
  last_published_pose_.pose.position.x = p_publish.x();
  last_published_pose_.pose.position.y = p_publish.y();
  last_published_pose_.pose.position.z = p_publish.z();
  last_published_pose_.pose.orientation.x = q_publish.x();
  last_published_pose_.pose.orientation.y = q_publish.y();
  last_published_pose_.pose.orientation.z = q_publish.z();
  last_published_pose_.pose.orientation.w = q_publish.w();

  desired_gripper_pose_publisher_.publish(last_published_pose_);
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
  CylindricalPoint pos_diff = this->getCylindricalPositionDifference();
  CylindricalPoint scaled_diff = this->scaleCylindricalPositionDifference(pos_diff);
  this->applyCylindricalPositionDifference(scaled_diff);
}

CylindricalPoint MotionControllerTeleoperation::getCylindricalPositionDifference() {
  geometry_msgs::Point init_hand_pos;
  init_hand_pos.x = initial_left_controller_pose_.pose.position.x - shoulder_position_.x;
  init_hand_pos.y = initial_left_controller_pose_.pose.position.y - shoulder_position_.y;
  init_hand_pos.z = initial_left_controller_pose_.pose.position.z - shoulder_position_.z;
  const CylindricalPoint init_hand_cpos = {
      std::hypot(init_hand_pos.x, init_hand_pos.y),
      std::atan2(init_hand_pos.y, init_hand_pos.x),
      init_hand_pos.z
  };

  geometry_msgs::Point curr_hand_pos;
  curr_hand_pos.x = current_left_controller_pose_.pose.position.x - shoulder_position_.x;
  curr_hand_pos.y = current_left_controller_pose_.pose.position.y - shoulder_position_.y;
  curr_hand_pos.z = current_left_controller_pose_.pose.position.z - shoulder_position_.z;
  const CylindricalPoint curr_hand_cpos = {
      std::hypot(curr_hand_pos.x, curr_hand_pos.y),
      std::atan2(curr_hand_pos.y, curr_hand_pos.x),
      curr_hand_pos.z
  };
  
  const CylindricalPoint pos_diff = {
      curr_hand_cpos.radius - init_hand_cpos.radius,
      this->restrictAngleWithinPI(
          curr_hand_cpos.yaw - init_hand_cpos.yaw
      ),
      curr_hand_cpos.height - init_hand_cpos.height
  };
  return pos_diff;
}

CylindricalPoint MotionControllerTeleoperation::scaleCylindricalPositionDifference(
    const CylindricalPoint& pos_diff) {
  CylindricalPoint scaled_diff;
  scaled_diff.radius = pos_diff.radius * gripper_position_radius_scale_;
  scaled_diff.yaw = pos_diff.yaw * gripper_position_yaw_scale_;
  scaled_diff.height = pos_diff.height * gripper_position_height_scale_;
  return scaled_diff;
}

void MotionControllerTeleoperation::applyCylindricalPositionDifference(
    const CylindricalPoint& scaled_diff) {
  // Calculate desired_gripper_position
  Eigen::Vector3d position_with_radius_height =
      this->getPositionWithRadiusAndHeightApplied(scaled_diff);
  Eigen::AngleAxisd yaw_rotation(scaled_diff.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d desired_gripper_position = yaw_rotation * position_with_radius_height;
  
  // Calculate desired_gripper_quaternion
  Eigen::Quaterniond initial_gripper_quaternion(
      initial_gripper_pose_.pose.orientation.w,
      initial_gripper_pose_.pose.orientation.x,
      initial_gripper_pose_.pose.orientation.y,
      initial_gripper_pose_.pose.orientation.z
  );
  Eigen::Quaterniond desired_gripper_quaternion
      = yaw_rotation * initial_gripper_quaternion;
  
  // Apply changes to the output
  desired_gripper_pose_.pose.position.x = desired_gripper_position.x();
  desired_gripper_pose_.pose.position.y = desired_gripper_position.y();
  desired_gripper_pose_.pose.position.z = desired_gripper_position.z();
  desired_gripper_pose_.pose.orientation.x = desired_gripper_quaternion.x();
  desired_gripper_pose_.pose.orientation.y= desired_gripper_quaternion.y();
  desired_gripper_pose_.pose.orientation.z = desired_gripper_quaternion.z();
  desired_gripper_pose_.pose.orientation.w = desired_gripper_quaternion.w();
}

Eigen::Vector3d MotionControllerTeleoperation::getPositionWithRadiusAndHeightApplied(
    const CylindricalPoint& scaled_diff) {
  const Eigen::Vector3d initial_gripper_position(
      initial_gripper_pose_.pose.position.x,
      initial_gripper_pose_.pose.position.y,
      initial_gripper_pose_.pose.position.z
  );

  // vector that points to initial_gripper_pose_ without considering z axis
  const Eigen::Vector2d init_xy(
      initial_gripper_position.x(),
      initial_gripper_position.y()
  );
  // unit vector with angle `control_view_angle_`
  const Eigen::Vector2d view_xy(
      std::cos(control_view_angle_),
      std::sin(control_view_angle_)
  );

  // Calculate and clamp radius
  const double initial_radius  = init_xy.dot(view_xy);
  const double clamped_radius = std::clamp(
      initial_radius + scaled_diff.radius,
      gripper_position_radius_min_,
      gripper_position_radius_max_) - initial_radius;
  // Calculate and clamp height
  const double clamped_height = std::clamp(
      initial_gripper_position.z() + scaled_diff.height,
      gripper_position_height_min_,
      gripper_position_height_max_
  );

  Eigen::Vector3d position_with_radius_height(
      initial_gripper_position.x() + clamped_radius * view_xy.x(),
      initial_gripper_position.y() + clamped_radius * view_xy.y(),
      clamped_height
  );
  return position_with_radius_height;
}

void MotionControllerTeleoperation::calculateDesiredGripperOrientation() {
  const RPY orient_diff = this->getOrientationDifference();
  const RPY scaled_diff = this->scaleOrientationDifference(orient_diff);
  this->applyOrientationDifference(scaled_diff);
}

RPY MotionControllerTeleoperation::getOrientationDifference() {
  RPY init_rpy; init_rpy.convertFromPoseStamped(initial_left_controller_pose_);
  RPY curr_rpy; curr_rpy.convertFromPoseStamped(current_left_controller_pose_);
  RPY orient_diff = {
      this->restrictAngleWithinPI(curr_rpy.roll - init_rpy.roll),
      this->restrictAngleWithinPI(curr_rpy.pitch - init_rpy.pitch),
      this->restrictAngleWithinPI(curr_rpy.yaw - init_rpy.yaw)
  };
  return orient_diff;
}

RPY MotionControllerTeleoperation::scaleOrientationDifference(const RPY& orient_diff) {
  RPY scaled_diff;
  scaled_diff.roll = orient_diff.roll * gripper_orientation_roll_scale_;
  scaled_diff.pitch = orient_diff.pitch * gripper_orientation_pitch_scale_;
  scaled_diff.yaw = orient_diff.yaw * gripper_orientation_yaw_scale_;
  return scaled_diff;
}

void MotionControllerTeleoperation::applyOrientationDifference(const RPY& scaled_diff) {
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

  const Eigen::Vector3d p_gripper_wrist(0.0, 0.0, -0.291);  // TM5-700 spec.
  const Eigen::Vector3d wrist_position = initial_gripper_position +
      (initial_gripper_rotation_matrix * p_gripper_wrist);

  const double clamped_yaw = std::clamp(
      scaled_diff.yaw,
      -gripper_orientation_yaw_limit_,
      gripper_orientation_yaw_limit_
  );
  const double clamped_pitch = std::clamp(
      scaled_diff.pitch,
      -gripper_orientation_pitch_limit_,
      gripper_orientation_pitch_limit_
  );
  const double clamped_roll = std::clamp(
      scaled_diff.roll,
      -gripper_orientation_roll_limit_,
      gripper_orientation_roll_limit_
  );

  Eigen::AngleAxisd yaw_rotation(clamped_yaw, Eigen::Vector3d::UnitZ());  // global
  Eigen::AngleAxisd pitch_rotation(clamped_pitch, Eigen::Vector3d::UnitX());  // local
  Eigen::AngleAxisd roll_rotation(-clamped_roll, Eigen::Vector3d::UnitZ());  // local
  // = global_rotation * initial_quat * local_rotation
  Eigen::Quaterniond desired_gripper_quaternion = yaw_rotation
      * initial_gripper_quaternion * pitch_rotation * roll_rotation;
  Eigen::Matrix3d desired_rotation_matrix = desired_gripper_quaternion.toRotationMatrix();
  Eigen::Vector3d desired_gripper_position = wrist_position -
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
