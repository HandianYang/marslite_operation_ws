#include "cartesian_control/cartesian_control_joystick.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

CartesianControlJoystick::CartesianControlJoystick(const ros::NodeHandle& nh)
  : nh_(nh), rate_(ros::Rate(25)), is_begin_teleoperation_(false)
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
    if (is_index_trigger_pressed_ && is_hand_trigger_pressed_) {
      if (is_begin_teleoperation_) {
        // ROS_INFO_STREAM("Begin the teleoperation...");
        previous_left_joy_pose_.pose = current_left_joy_pose_.pose;
        is_begin_teleoperation_ = false;
      }

      relative_gripper_pose_ = this->getScaledRelativeGripperPose();
      target_gripper_pose_ = this->getTargetGripperPose();
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

void CartesianControlJoystick::initializePublishersAndSubscribers()
{
  target_frame_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 10);
  left_joy_pose_sub_ = nh_.subscribe("/unity/joy_pose/left", 10, &CartesianControlJoystick::leftJoyPoseCallback, this);
  left_joy_sub_ = nh_.subscribe("/unity/joy/left", 10, &CartesianControlJoystick::leftJoyCallback, this);
}

void CartesianControlJoystick::parseParameters()
{
  ros::NodeHandle pnh("~");
  pnh.param("position_scale", position_scale_, 0.8);
  pnh.param("orientation_scale", orientation_scale_, 0.8);
}

RPY CartesianControlJoystick::getRPY(const geometry_msgs::PoseStamped& pose)
{
  RPY rpy;
  double x = pose.pose.orientation.x;
  double y = pose.pose.orientation.y;
  double z = pose.pose.orientation.z;
  double w = pose.pose.orientation.w;

  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  m.getRPY(rpy.roll, rpy.pitch, rpy.yaw);

  return rpy;
}

void CartesianControlJoystick::setInitialGripperPose()
{
  initial_gripper_pose_.header.frame_id = "base_link";
  initial_gripper_pose_.pose.position.x = 0.712;
  initial_gripper_pose_.pose.position.y = -0.122;
  initial_gripper_pose_.pose.position.z = 1.043;
  initial_gripper_pose_.pose.orientation.x = 0.5;
  initial_gripper_pose_.pose.orientation.y = 0.5;
  initial_gripper_pose_.pose.orientation.z = 0.5;
  initial_gripper_pose_.pose.orientation.w = 0.5;
  target_gripper_pose_ = initial_gripper_pose_;
  target_frame_pub_.publish(target_gripper_pose_);
}

geometry_msgs::PoseStamped CartesianControlJoystick::getScaledRelativeGripperPose()
{
  geometry_msgs::PoseStamped relative_gripper_pose;
  relative_gripper_pose.header.frame_id = "base_link";
  relative_gripper_pose.pose.position.x = (current_left_joy_pose_.pose.position.x - previous_left_joy_pose_.pose.position.x) * position_scale_;
  relative_gripper_pose.pose.position.y = (current_left_joy_pose_.pose.position.y - previous_left_joy_pose_.pose.position.y) * position_scale_;
  relative_gripper_pose.pose.position.z = (current_left_joy_pose_.pose.position.z - previous_left_joy_pose_.pose.position.z) * position_scale_;

  RPY previous_left_joy_rpy = getRPY(previous_left_joy_pose_);
  RPY current_left_joy_rpy = getRPY(current_left_joy_pose_);
  RPY difference = {
    current_left_joy_rpy.roll - previous_left_joy_rpy.roll,
    current_left_joy_rpy.pitch - previous_left_joy_rpy.pitch,
    current_left_joy_rpy.yaw - previous_left_joy_rpy.yaw
  };
  tf2::Quaternion q;
  q.setRPY(
      difference.roll * orientation_scale_,
      difference.pitch * orientation_scale_,
      difference.yaw * orientation_scale_
  );
  relative_gripper_pose.pose.orientation.x = q.x();
  relative_gripper_pose.pose.orientation.y = q.y();
  relative_gripper_pose.pose.orientation.z = q.z();
  relative_gripper_pose.pose.orientation.w = q.w();
  return relative_gripper_pose;
}

geometry_msgs::PoseStamped CartesianControlJoystick::getTargetGripperPose()
{
  geometry_msgs::PoseStamped target_gripper_pose;
  target_gripper_pose.header.frame_id = "base_link";

  target_gripper_pose.pose.position.x = initial_gripper_pose_.pose.position.x + relative_gripper_pose_.pose.position.x;
  target_gripper_pose.pose.position.y = initial_gripper_pose_.pose.position.y + relative_gripper_pose_.pose.position.y;
  target_gripper_pose.pose.position.z = initial_gripper_pose_.pose.position.z + relative_gripper_pose_.pose.position.z;
  
  tf2::Quaternion initial_q(
      initial_gripper_pose_.pose.orientation.x,
      initial_gripper_pose_.pose.orientation.y,
      initial_gripper_pose_.pose.orientation.z,
      initial_gripper_pose_.pose.orientation.w
  );

  tf2::Quaternion relative_q(
      relative_gripper_pose_.pose.orientation.x,
      relative_gripper_pose_.pose.orientation.y,
      relative_gripper_pose_.pose.orientation.z,
      relative_gripper_pose_.pose.orientation.w
  );
  tf2::Quaternion target_q = relative_q * initial_q;
  target_q.normalize();

  target_gripper_pose.pose.orientation.x = target_q.x();
  target_gripper_pose.pose.orientation.y = target_q.y();
  target_gripper_pose.pose.orientation.z = target_q.z();
  target_gripper_pose.pose.orientation.w = target_q.w();
  return target_gripper_pose;
}

void CartesianControlJoystick::leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_left_joy_pose_.pose = msg->pose;
}

void CartesianControlJoystick::leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // axes[0] and axes[1] are for mobile platform teleoperation
  switch (msg->axes.size()) {
    case 4:
      // [3] primary hand trigger
      is_hand_trigger_pressed_ = (msg->axes[3] > kTriggerThreshold);
    case 3:
      // [2] primary index trigger
      is_index_trigger_pressed_ = (msg->axes[2] > kTriggerThreshold);
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick axes (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }

  switch (msg->buttons.size()) {
    case 2:
      // [1] Y
      is_button_Y_pressed_ = (msg->buttons[1] == 1);
    case 1:
      // [0] X
      static ros::Time button_X_pressed_time;
      if (msg->buttons[0] == 1) {
        if (button_X_pressed_time.isZero()) {
          button_X_pressed_time = ros::Time::now();
        } else if ((ros::Time::now() - button_X_pressed_time).toSec() > 3.0) {
          is_button_X_pressed_over_3s_ = true;
        }
      } else {
        button_X_pressed_time = ros::Time(0);
      }
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick buttons (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }
}