#include "cartesian_control/joystick_teleoperation.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

JoystickTeleoperationWrapper::JoystickTeleoperationWrapper(const ros::NodeHandle& nh)
  : nh_(nh), rate_(ros::Rate(10)), use_shared_controller_(false),
    is_position_change_enabled_(false), is_orientation_change_enabled_(false) {
  this->parseParameters();
  this->initializePublishers();
  this->initializeSubscribers();
  current_gripper_pose_ = tf2_listener_.lookupTransformInPoseStamped("base_link", "tm_gripper");
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void JoystickTeleoperationWrapper::teleoperate() {
  while (nh_.ok()) {
    this->calculateDesiredGripperDisplacement();
    if (use_shared_controller_) {
      // shared control -> publish gripper displacement to shared controller
      this->publishDesiredGripperDIsplacement();
    } else {
      // pure teleoperation -> directly publish the desired gripper pose
      this->calculateDesiredGripperPose();
      this->publishDesiredGripperPose();
    }

    rate_.sleep();
    ros::spinOnce();
  }
}

void JoystickTeleoperationWrapper::calculateDesiredGripperDisplacement() {
  if (is_position_change_enabled_) {
    geometry_msgs::Point position_difference = this->getPositionDifference();
    geometry_msgs::Point scaled_position_difference = this->scalePositionDifference(position_difference);
    desired_gripper_displacement_.pose.position = scaled_position_difference;
  } else {
    desired_gripper_displacement_.pose.position = geometry_msgs::Point();
  }

  if (is_orientation_change_enabled_) {
    RPY rpy_difference = this->getRPYDifference();
    RPY scaled_rpy_difference = this->scaleAndTransformRPYDifference(rpy_difference);
    desired_gripper_displacement_.pose.orientation = this->convertRPYToQuaternion(scaled_rpy_difference);
  } else {
    desired_gripper_displacement_.pose.orientation = geometry_msgs::Quaternion();
    desired_gripper_displacement_.pose.orientation.w = 1.0;
  }
}

void JoystickTeleoperationWrapper::calculateDesiredGripperPose() {
  if (is_position_change_enabled_ || is_orientation_change_enabled_) {
    desired_gripper_pose_.header.frame_id = "base_link";
    desired_gripper_pose_.header.stamp = ros::Time::now();
    desired_gripper_pose_.pose.position = this->applyPositionDisplacement();
    desired_gripper_pose_.pose.orientation = this->applyOrientationDisplacement();
    current_gripper_pose_ = desired_gripper_pose_;
  } else {
    desired_gripper_pose_ = tf2_listener_.lookupTransformInPoseStamped("base_link", "tm_gripper");
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

// 
// initialization
// 

void JoystickTeleoperationWrapper::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("position_scale", position_scale_, 0.8);
  pnh.param("orientation_scale", orientation_scale_, 0.8);
  pnh.param("use_shared_controller", use_shared_controller_, false);
  ROS_INFO_STREAM("Parameters: " 
      << "\n * position_scale: " << position_scale_
      << "\n * orientation_scale: " << orientation_scale_
      << "\n * use_shared_controller: " << use_shared_controller_);
}

void JoystickTeleoperationWrapper::initializePublishers() {
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/target_frame", 1);
  desired_gripper_displacement_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/cartesian_control/user_desired_gripper_displacement", 1);
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/cartesian_control/user_desired_gripper_status", 1);
}

void JoystickTeleoperationWrapper::initializeSubscribers() {
  left_joy_pose_subscriber_ = nh_.subscribe("/unity/joy_pose/left", 1,
      &JoystickTeleoperationWrapper::leftJoyPoseCallback, this);
  left_joy_subscriber_ = nh_.subscribe("/unity/joy/left", 1,
      &JoystickTeleoperationWrapper::leftJoyCallback, this);
}

//
// utility operations (supports calculateDesiredGripperDisplacement())
//

geometry_msgs::Point JoystickTeleoperationWrapper::getPositionDifference() const {
  geometry_msgs::Point position_difference;
  position_difference.x = current_left_joy_pose_.pose.position.x - previous_left_joy_pose_.pose.position.x;
  position_difference.y = current_left_joy_pose_.pose.position.y - previous_left_joy_pose_.pose.position.y;
  position_difference.z = current_left_joy_pose_.pose.position.z - previous_left_joy_pose_.pose.position.z;
  return position_difference;
}

geometry_msgs::Point JoystickTeleoperationWrapper::scalePositionDifference(
    const geometry_msgs::Point& position_difference) const {
  geometry_msgs::Point scaled_position_difference;
  scaled_position_difference.x = position_difference.x * position_scale_;
  scaled_position_difference.y = position_difference.y * position_scale_;
  scaled_position_difference.z = position_difference.z * position_scale_;
  return scaled_position_difference;
}

RPY JoystickTeleoperationWrapper::getRPYDifference() const {
  RPY previous_left_joy_rpy = getRPYFromPose(previous_left_joy_pose_);
  RPY current_left_joy_rpy = getRPYFromPose(current_left_joy_pose_);
  RPY rpy_difference = {
      this->restrictAngleWithinPI(current_left_joy_rpy.roll - previous_left_joy_rpy.roll),
      this->restrictAngleWithinPI(current_left_joy_rpy.pitch - previous_left_joy_rpy.pitch),
      this->restrictAngleWithinPI(current_left_joy_rpy.yaw - previous_left_joy_rpy.yaw)
  };
  return rpy_difference;
}

RPY JoystickTeleoperationWrapper::getRPYFromPose(const geometry_msgs::PoseStamped& pose) const {
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

double JoystickTeleoperationWrapper::restrictAngleWithinPI(const double& angle) const {
  if (angle > M_PI)
    return angle - 2 * M_PI;
  else if (angle < -M_PI)
    return angle + 2 * M_PI;
  return angle;
}

RPY JoystickTeleoperationWrapper::scaleAndTransformRPYDifference(const RPY& rpy_difference) const {
  // [NOTE] The transformation from `/base_link` to `/tm_gripper` is applied here`:
  //   | /base_link | /tm_gripper |
  //   |------------|-------------|
  //   |   +-roll   |   -+pitch   |
  //   |   +-pitch  |   +-roll    |
  //   |   +-yaw    |   +-yaw     |
  //   |------------|-------------|
  RPY scaled_rpy_difference;
  scaled_rpy_difference.roll = rpy_difference.pitch * orientation_scale_;
  scaled_rpy_difference.pitch = 0;
  // [NOTE] The pitch of the transformed RPY is disabled because we don't need roll rotation
  //   for the gripper (w.r.t. `base_link`)`. Uncomment the line below if you want
  //   to enable it.
  //  
  // scaled_rpy_difference.pitch = -rpy_difference.roll * orientation_scale_;
  scaled_rpy_difference.yaw = rpy_difference.yaw * orientation_scale_;
  return scaled_rpy_difference;
}

geometry_msgs::Quaternion JoystickTeleoperationWrapper::convertRPYToQuaternion(const RPY& rpy) const {
  tf2::Quaternion q;
  q.setRPY(rpy.roll, rpy.pitch, rpy.yaw);
  q.normalize();
  geometry_msgs::Quaternion quaternion;
  quaternion.x = q.x();
  quaternion.y = q.y();
  quaternion.z = q.z();
  quaternion.w = q.w();
  return quaternion;
}

geometry_msgs::Point JoystickTeleoperationWrapper::applyPositionDisplacement() const {
  geometry_msgs::Point desired_position;
  desired_position.x = current_gripper_pose_.pose.position.x + desired_gripper_displacement_.pose.position.x;
  desired_position.y = current_gripper_pose_.pose.position.y + desired_gripper_displacement_.pose.position.y;
  desired_position.z = current_gripper_pose_.pose.position.z + desired_gripper_displacement_.pose.position.z;
  return desired_position;
}

geometry_msgs::Quaternion JoystickTeleoperationWrapper::applyOrientationDisplacement() const {
  tf2::Quaternion current_gripper_pose_quaternion(
      current_gripper_pose_.pose.orientation.x,
      current_gripper_pose_.pose.orientation.y,
      current_gripper_pose_.pose.orientation.z,
      current_gripper_pose_.pose.orientation.w
  );
  tf2::Quaternion desired_gripper_displacement_quaternion(
      desired_gripper_displacement_.pose.orientation.x,
      desired_gripper_displacement_.pose.orientation.y,
      desired_gripper_displacement_.pose.orientation.z,
      desired_gripper_displacement_.pose.orientation.w
  );
  tf2::Quaternion result_quaternion
      = current_gripper_pose_quaternion * desired_gripper_displacement_quaternion;
  result_quaternion.normalize();
  
  geometry_msgs::Quaternion result_geometry_quaternion;
  result_geometry_quaternion.x = result_quaternion.x();
  result_geometry_quaternion.y = result_quaternion.y();
  result_geometry_quaternion.z = result_quaternion.z();
  result_geometry_quaternion.w = result_quaternion.w();
  return result_geometry_quaternion;
}


//
// callbacks
//

void JoystickTeleoperationWrapper::leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  previous_left_joy_pose_ = current_left_joy_pose_;
  current_left_joy_pose_.pose = msg->pose;
}

void JoystickTeleoperationWrapper::leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  // axes[0] and axes[1] are for mobile platform teleoperation
  switch (msg->axes.size()) {
    case 4:
      // [3] primary hand trigger
      is_orientation_change_enabled_ = (msg->axes[3] > kTriggerThreshold);
    case 3:
      // [2] primary index trigger
      is_position_change_enabled_ = (msg->axes[2] > kTriggerThreshold);
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick axes (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }

  switch (msg->buttons.size()) {
    case 2:
      // [1] Y button: 
      if (msg->buttons[1] == 1) {
        // TODO: Implement record_signal()
      }
    case 1:
      // [0] X button: Close/Open the gripper
      if (msg->buttons[0] == 1) {
        desired_gripper_status_.data = !desired_gripper_status_.data;
        desired_gripper_status_publisher_.publish(desired_gripper_status_);
      }
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick buttons (%lu).", msg->buttons.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }
}