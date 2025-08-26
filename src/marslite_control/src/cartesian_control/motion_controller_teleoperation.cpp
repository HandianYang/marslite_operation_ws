#include "cartesian_control/motion_controller_teleoperation.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

MotionControllerTeleoperation::MotionControllerTeleoperation(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(25)), is_begin_teleoperation_(false),
      is_position_change_enabled_(false), is_orientation_change_enabled_(false) {
  this->parseParameters();
  this->initializePublishers();
  this->initializeSubscribers();
  this->setInitialGripperPose();
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void MotionControllerTeleoperation::teleoperate() {
  while (nh_.ok()) {
    this->lookupCurrentGripperTransform();
    if (is_position_change_enabled_ || is_orientation_change_enabled_) {
      if (is_begin_teleoperation_) {
        initial_left_controller_pose_.pose = current_left_controller_pose_.pose;
        is_begin_teleoperation_ = false;
      }
      this->calculateDesiredGripperPose();
    } else {
      this->resetPositionalMovement();
      this->resetOrientationalMovement();
      initial_gripper_pose_ = desired_gripper_pose_;
      is_begin_teleoperation_ = true;
    }
    this->publishDesiredGripperPose();
    this->publishMobilePlatformVelocity();

    rate_.sleep();
    ros::spinOnce();
  }
}

void MotionControllerTeleoperation::calculateDesiredGripperPose() {
  if (is_position_change_enabled_) {
    geometry_msgs::Vector3 position_difference = this->getPositionDifference();
    geometry_msgs::Vector3 scaled_position_difference;
    scaled_position_difference.x = position_difference.x * position_scale_;
    scaled_position_difference.y = position_difference.y * position_scale_;
    scaled_position_difference.z = position_difference.z * position_scale_;

    desired_gripper_pose_.pose.position.x = initial_gripper_pose_.pose.position.x + scaled_position_difference.x;
    desired_gripper_pose_.pose.position.y = initial_gripper_pose_.pose.position.y + scaled_position_difference.y;
    desired_gripper_pose_.pose.position.z = initial_gripper_pose_.pose.position.z + scaled_position_difference.z;
  }
  else {
    this->resetPositionalMovement();
  }

  if (is_orientation_change_enabled_) {
    RPY orientation_difference = this->getOrientationDifference();

    // [NOTE] The transformation from `/base_link` to `/tm_gripper` is applied here`:
    //   | /base_link | /tm_gripper |
    //   |------------|-------------|
    //   |   +-roll   |   -+pitch   |
    //   |   +-pitch  |   +-roll    |
    //   |   +-yaw    |   +-yaw     |
    //
    RPY scaled_orientation_difference;
    scaled_orientation_difference.roll = orientation_difference.pitch * orientation_scale_;
    scaled_orientation_difference.pitch = 0;
    // [NOTE] The pitch of the transformed RPY is disabled because we don't need roll rotation
    //   for the gripper (w.r.t. `base_link`)`. Uncomment the line below if you want
    //   to enable it.
    //  
    // scaled_orientation_difference.pitch = -orientation_difference.roll * orientation_scale_;
    scaled_orientation_difference.yaw = orientation_difference.yaw * orientation_scale_;

    RPY initial_gripper_rpy = getRPYFromPose(initial_gripper_pose_);
    RPY target_gripper_rpy = {
        this->restrictAngleWithinPI(initial_gripper_rpy.roll + scaled_orientation_difference.roll),
        this->restrictAngleWithinPI(initial_gripper_rpy.pitch + scaled_orientation_difference.pitch),
        this->restrictAngleWithinPI(initial_gripper_rpy.yaw + scaled_orientation_difference.yaw)
    };

    tf2::Quaternion target_gripper_quaternion;
    target_gripper_quaternion.setRPY(
        target_gripper_rpy.roll,
        target_gripper_rpy.pitch,
        target_gripper_rpy.yaw
    );
    target_gripper_quaternion.normalize();
    desired_gripper_pose_.pose.orientation.x = target_gripper_quaternion.x();
    desired_gripper_pose_.pose.orientation.y = target_gripper_quaternion.y();
    desired_gripper_pose_.pose.orientation.z = target_gripper_quaternion.z();
    desired_gripper_pose_.pose.orientation.w = target_gripper_quaternion.w();
  }
  else {
    this->resetOrientationalMovement();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

// 
// initialization
// 

void MotionControllerTeleoperation::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("position_scale", position_scale_, 0.8);
  pnh.param("orientation_scale", orientation_scale_, 0.8);
  pnh.param("use_shared_controller", use_shared_controller_, false);
  ROS_INFO_STREAM("Parameters: " 
      << "\n * position_scale: " << position_scale_
      << "\n * orientation_scale: " << orientation_scale_
      << "\n * use_shared_controller: " << use_shared_controller_);
}

void MotionControllerTeleoperation::initializePublishers() {
  // The desired gripper pose will be published to different topics according
  // to the `use_shared_controller_` parameter.
  // - use_shared_controller_ = true: published to shared controller
  // - use_shared_controller_ = false: published to the robot
  const std::string gripper_pose_topic = use_shared_controller_ ?
      "/marslite_control/user_desired_gripper_pose" :
      "/cartesian_control/target_frame";
  const std::string gripper_status_topic = use_shared_controller_ ?
      "/marslite_control/user_desired_gripper_status" :
      "/gripper/cmd_gripper";
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(gripper_pose_topic, 1);
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(gripper_status_topic, 1);
  record_signal_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/marslite_control/record_signal", 1);
  mobile_platform_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(
      "/mob_plat/cmd_vel", 1);
}

void MotionControllerTeleoperation::initializeSubscribers() {
  left_controller_pose_subscriber_ = nh_.subscribe("/unity/controller/left/pose", 1,
      &MotionControllerTeleoperation::leftControllerPoseCallback, this);
  left_controller_joy_subscriber_ = nh_.subscribe("/unity/controller/left/joy", 1,
      &MotionControllerTeleoperation::leftControllerJoyCallback, this);
}

void MotionControllerTeleoperation::setInitialGripperPose() {
  initial_gripper_pose_ = desired_gripper_pose_ = kInitialGripperPose;
}

//
// utility operations (supports calculateDesiredGripperPose())
//

geometry_msgs::Vector3 MotionControllerTeleoperation::getPositionDifference() {
  geometry_msgs::Vector3 position_difference;
  position_difference.x = current_left_controller_pose_.pose.position.x - initial_left_controller_pose_.pose.position.x;
  position_difference.y = current_left_controller_pose_.pose.position.y - initial_left_controller_pose_.pose.position.y;
  position_difference.z = current_left_controller_pose_.pose.position.z - initial_left_controller_pose_.pose.position.z;
  return position_difference;
}

RPY MotionControllerTeleoperation::getOrientationDifference() {
  RPY initial_left_controller_rpy = getRPYFromPose(initial_left_controller_pose_);
  RPY current_left_controller_rpy = getRPYFromPose(current_left_controller_pose_);
  RPY orientation_difference = {
      this->restrictAngleWithinPI(current_left_controller_rpy.roll - initial_left_controller_rpy.roll),
      this->restrictAngleWithinPI(current_left_controller_rpy.pitch - initial_left_controller_rpy.pitch),
      this->restrictAngleWithinPI(current_left_controller_rpy.yaw - initial_left_controller_rpy.yaw)
  };
  return orientation_difference;
}

RPY MotionControllerTeleoperation::getRPYFromPose(const geometry_msgs::PoseStamped& pose) {
  RPY rpy;
  const double x = pose.pose.orientation.x;
  const double y = pose.pose.orientation.y;
  const double z = pose.pose.orientation.z;
  const double w = pose.pose.orientation.w;

  tf2::Quaternion q(x, y, z, w);
  if (q.length2() == 0.0) {
    ROS_ERROR("Invalid quaternion with zero length encountered in getRPYFromPose.");
    rpy.roll = rpy.pitch = rpy.yaw = 0.0;
    return rpy;
  }

  q.normalize();
  tf2::Matrix3x3 m(q);
  m.getRPY(rpy.roll, rpy.pitch, rpy.yaw);

  return rpy;
}

double MotionControllerTeleoperation::restrictAngleWithinPI(const double& angle) {
  if (angle > M_PI)
    return angle - 2 * M_PI;
  else if (angle < -M_PI)
    return angle + 2 * M_PI;
  return angle;
}

//
// callbacks
//

void MotionControllerTeleoperation::leftControllerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_left_controller_pose_.pose = msg->pose;
}

void MotionControllerTeleoperation::leftControllerJoyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  switch (msg->axes.size()) {
    case 4:
      // [3] primary hand trigger: Enable/disable orientational change
      is_orientation_change_enabled_ = (msg->axes[3] > kTriggerThreshold);
    case 3:
      // [2] primary index trigger: Enable/disable positional change
      is_position_change_enabled_ = (msg->axes[2] > kTriggerThreshold);
    case 2:
      // [1] horizontal axis: Turn left/right
      mobile_platform_velocity_.angular.z = msg->axes[1] * 0.2;
    case 1:
      // [0] vertical axis: Move forward/backward
      mobile_platform_velocity_.linear.x = msg->axes[0] * 0.3;
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
  if ((current_time - last_toggle_time).toSec() > 0.5) {
    desired_gripper_status_.data = !desired_gripper_status_.data;
    desired_gripper_status_publisher_.publish(desired_gripper_status_);
    last_toggle_time = current_time;
  }
}