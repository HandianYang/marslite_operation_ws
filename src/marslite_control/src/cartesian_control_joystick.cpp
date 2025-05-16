#include "cartesian_control/cartesian_control_joystick.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

CartesianControlJoystick::CartesianControlJoystick(const ros::NodeHandle& nh)
  : nh_(nh), rate_(ros::Rate(25)), is_begin_teleoperation_(false),
    is_position_change_enabled_(false), is_orientation_change_enabled_(false)
{
  this->parseParameters();
  this->initializePublishersAndSubscribers();
  this->setInitialGripperPose();
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void CartesianControlJoystick::run()
{
  while (ros::ok()) {
    if (is_position_change_enabled_ || is_orientation_change_enabled_) {
      if (is_begin_teleoperation_) {
        // ROS_INFO_STREAM("Begin the teleoperation...");
        initial_left_joy_pose_.pose = current_left_joy_pose_.pose;
        this->obtainInitialLeftJoyPose();
        is_begin_teleoperation_ = false;
      }

      this->calculateTargetGripperPose();
      target_frame_pub_.publish(target_gripper_pose_);
    } else {
      is_begin_teleoperation_ = true;
      initial_gripper_pose_ = target_gripper_pose_;
    }
    
    rate_.sleep();
    ros::spinOnce();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void CartesianControlJoystick::parseParameters()
{
  ros::NodeHandle pnh("~");
  pnh.param("position_scale", position_scale_, 0.8);
  pnh.param("orientation_scale", orientation_scale_, 0.3);
}

void CartesianControlJoystick::initializePublishersAndSubscribers()
{
  target_frame_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 10);
  left_joy_pose_sub_ = nh_.subscribe("/unity/joy_pose/left", 10, &CartesianControlJoystick::leftJoyPoseCallback, this);
  left_joy_sub_ = nh_.subscribe("/unity/joy/left", 10, &CartesianControlJoystick::leftJoyCallback, this);
}

void CartesianControlJoystick::setInitialGripperPose()
{
  initial_gripper_pose_ = target_gripper_pose_ = kInitialGripperPose;
}

RPY CartesianControlJoystick::getRPYFromPose(const geometry_msgs::PoseStamped& pose)
{
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

double CartesianControlJoystick::restrictAngleWithinPI(const double& angle)
{
  if (angle > M_PI)
    return angle - 2 * M_PI;
  else if (angle < -M_PI)
    return angle + 2 * M_PI;
  return angle;
}

tf2::Quaternion CartesianControlJoystick::convertQuaternionFromJoystickToWorld(const tf2::Quaternion& quaternion_difference)
{
  return (initial_left_joy_quaternion_.inverse() * quaternion_difference * initial_left_joy_quaternion_).normalize();
}

void CartesianControlJoystick::obtainInitialLeftJoyPose()
{
  initial_left_joy_quaternion_ = tf2::Quaternion(
    initial_left_joy_pose_.pose.orientation.x,
    initial_left_joy_pose_.pose.orientation.y,
    initial_left_joy_pose_.pose.orientation.z,
    initial_left_joy_pose_.pose.orientation.w
  );
}

geometry_msgs::Vector3 CartesianControlJoystick::getPositionDifference()
{
  geometry_msgs::Vector3 position_difference;
  position_difference.x = current_left_joy_pose_.pose.position.x - initial_left_joy_pose_.pose.position.x;
  position_difference.y = current_left_joy_pose_.pose.position.y - initial_left_joy_pose_.pose.position.y;
  position_difference.z = current_left_joy_pose_.pose.position.z - initial_left_joy_pose_.pose.position.z;
  return position_difference;
}

RPY CartesianControlJoystick::getOrientationDifference()
{
  RPY initial_left_joy_rpy = getRPYFromPose(initial_left_joy_pose_);
  RPY current_left_joy_rpy = getRPYFromPose(current_left_joy_pose_);
  RPY orientation_difference = {
      this->restrictAngleWithinPI(current_left_joy_rpy.roll - initial_left_joy_rpy.roll),
      this->restrictAngleWithinPI(current_left_joy_rpy.pitch - initial_left_joy_rpy.pitch),
      this->restrictAngleWithinPI(current_left_joy_rpy.yaw - initial_left_joy_rpy.yaw)
  };
  return orientation_difference;
}

void CartesianControlJoystick::calculateTargetGripperPose()
{
  if (is_position_change_enabled_) {
    geometry_msgs::Vector3 position_difference = this->getPositionDifference();
    geometry_msgs::Vector3 scaled_position_difference;
    scaled_position_difference.x = position_difference.x * position_scale_;
    scaled_position_difference.y = position_difference.y * position_scale_;
    scaled_position_difference.z = position_difference.z * position_scale_;

    target_gripper_pose_.pose.position.x = initial_gripper_pose_.pose.position.x + scaled_position_difference.x;
    target_gripper_pose_.pose.position.y = initial_gripper_pose_.pose.position.y + scaled_position_difference.y;
    target_gripper_pose_.pose.position.z = initial_gripper_pose_.pose.position.z + scaled_position_difference.z;
  }

  if (is_orientation_change_enabled_) {
    RPY orientation_difference = this->getOrientationDifference();
    // ROS_INFO_STREAM("Orientation Difference (RPY): ["
    //         << orientation_difference.roll << ", "
    //         << orientation_difference.pitch << ", "
    //         << orientation_difference.yaw << "]");
    RPY scaled_orientation_difference;
    // scaled_orientation_difference.roll = orientation_difference.roll * orientation_scale_;
    // scaled_orientation_difference.pitch = orientation_difference.pitch * orientation_scale_;
    // scaled_orientation_difference.yaw = orientation_difference.yaw * orientation_scale_;
    scaled_orientation_difference.roll = orientation_difference.pitch * orientation_scale_;
    scaled_orientation_difference.pitch = -orientation_difference.roll * orientation_scale_;
    scaled_orientation_difference.yaw = orientation_difference.yaw * orientation_scale_;

    RPY initial_left_joy_rpy = getRPYFromPose(initial_left_joy_pose_);
    RPY target_gripper_rpy = {
        this->restrictAngleWithinPI(initial_left_joy_rpy.roll + scaled_orientation_difference.roll),
        this->restrictAngleWithinPI(initial_left_joy_rpy.pitch + scaled_orientation_difference.pitch),
        this->restrictAngleWithinPI(initial_left_joy_rpy.yaw + scaled_orientation_difference.yaw)
    };
    tf2::Quaternion target_gripper_quaternion;
    target_gripper_quaternion.setRPY(
        target_gripper_rpy.roll,
        target_gripper_rpy.pitch,
        target_gripper_rpy.yaw
    );
    // target_gripper_quaternion = kBaseLinkToTMTipLinkQuaternion * target_gripper_quaternion;
    // target_gripper_quaternion = tf2::Quaternion(0, 0, 0.707, 0.707) * target_gripper_quaternion;
    target_gripper_quaternion.normalize();

    // tf2::Quaternion scaled_quaternion_difference;
    // scaled_quaternion_difference.setRPY(
    //     scaled_orientation_difference.roll,
    //     scaled_orientation_difference.pitch,
    //     scaled_orientation_difference.yaw
    // );
    // scaled_quaternion_difference.normalize();
    // tf2::Quaternion target_gripper_quaternion = scaled_quaternion_difference * initial_left_joy_quaternion_;
    // target_gripper_quaternion.normalize();
    // tf2::Quaternion scaled_quaternion_difference_world =
    //     this->convertQuaternionFromJoystickToWorld(scaled_quaternion_difference);
    // tf2::Quaternion target_gripper_quaternion = scaled_quaternion_difference_world * initial_left_joy_quaternion_;
    target_gripper_pose_.pose.orientation.x = target_gripper_quaternion.x();
    target_gripper_pose_.pose.orientation.y = target_gripper_quaternion.y();
    target_gripper_pose_.pose.orientation.z = target_gripper_quaternion.z();
    target_gripper_pose_.pose.orientation.w = target_gripper_quaternion.w();
  }
}

void CartesianControlJoystick::leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // current_left_joy_pose_.pose = msg->pose;

  tf2::Quaternion current_left_joy_quaternion(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  );

  tf2::Quaternion base_link_to_tm_tip_link(0.5, 0.5, 0.5, 0.5);
  current_left_joy_quaternion = kBaseLinkToTMTipLinkQuaternion * current_left_joy_quaternion;
  current_left_joy_quaternion.normalize();

  current_left_joy_pose_.header = msg->header;
  current_left_joy_pose_.pose.position.x = msg->pose.position.x;
  current_left_joy_pose_.pose.position.y = msg->pose.position.y;
  current_left_joy_pose_.pose.position.z = msg->pose.position.z;
  current_left_joy_pose_.pose.orientation.x = current_left_joy_quaternion.x();
  current_left_joy_pose_.pose.orientation.y = current_left_joy_quaternion.y();
  current_left_joy_pose_.pose.orientation.z = current_left_joy_quaternion.z();
  current_left_joy_pose_.pose.orientation.w = current_left_joy_quaternion.w();

  // RPY current_rpy = getRPYFromPose(current_left_joy_pose_);
  // ROS_INFO_STREAM("Current Position: ["
  //                 << current_left_joy_pose_.pose.position.x << ", "
  //                 << current_left_joy_pose_.pose.position.y << ", "
  //                 << current_left_joy_pose_.pose.position.z << "]");
  // ROS_INFO_STREAM("Current Orientation (Quaternion): ["
  //                 << current_left_joy_pose_.pose.orientation.x << ", "
  //                 << current_left_joy_pose_.pose.orientation.y << ", "
  //                 << current_left_joy_pose_.pose.orientation.z << ", "
  //                 << current_left_joy_pose_.pose.orientation.w << "]");
}

void CartesianControlJoystick::leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
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
      // [1] Y
      if (msg->buttons[1] == 1) {
        target_gripper_pose_ = kInitialGripperPose;
        target_frame_pub_.publish(target_gripper_pose_);
      }
    case 1:
      // [0] X
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick buttons (%lu).", msg->buttons.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }
}