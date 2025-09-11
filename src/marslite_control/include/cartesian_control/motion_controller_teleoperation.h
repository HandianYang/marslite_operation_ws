#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "cartesian_control/kinematics_constants.h"
#include "utils/rpy.h"
#include "utils/tf2_listener_wrapper.h"

enum GripperDirection {
  FRONT=0,
  LEFT=1,
  RIGHT=2
};

const double kTriggerThreshold = 0.95;
const double kPositionTolerance = 1e-3;
const double kOrientationTolerance = 1e-3;

class MotionControllerTeleoperation {
 public:
  explicit MotionControllerTeleoperation(const ros::NodeHandle& nh = ros::NodeHandle());

  inline void resetToFrontPose() {
    this->teleoperateToPose(kFrontInitialGripperPose);
    this->setGripperDirection(GripperDirection::FRONT);
  }

  inline void resetToLeftPose() {
    this->teleoperateToPose(kLeftInitialGripperPose);
    this->setGripperDirection(GripperDirection::LEFT);
  }

  inline void resetToRightPose() {
    this->teleoperateToPose(kRightInitialGripperPose);
    this->setGripperDirection(GripperDirection::RIGHT);
  }

  void teleoperateToPose(const geometry_msgs::PoseStamped& target_pose);

  inline void setGripperDirection(const GripperDirection& direction = GripperDirection::FRONT) {
    gripper_direction_ = direction;
  }

  void run();

 private:
  // initialization
  void parseParameters();
  void initializePublishers();
  void initializeSubscribers();

  // utility operations (supports run())

  void lookupCurrentGripperPose();
  
  inline const bool isAnySafetyButtonPressed() const {
    return is_position_change_enabled_ || is_orientation_change_enabled_;
  }
  
  inline void initializeLeftControllerPose() {
    initial_left_controller_pose_.pose = current_left_controller_pose_.pose;
  }

  void calculateDesiredGripperPose();

  inline void resetPositionalMovement() {
    desired_gripper_pose_.pose.position = current_gripper_pose_.pose.position;
  }

  inline void resetOrientationalMovement() {
    desired_gripper_pose_.pose.orientation = current_gripper_pose_.pose.orientation;
  }

  inline void publishDesiredGripperPose() {
    gripper_pose_publisher_.publish(desired_gripper_pose_);
  }

  inline void publishMobilePlatformVelocity() {
    mobile_platform_velocity_publisher_.publish(mobile_platform_velocity_);
  }

  const bool targetPoseIsReached(const geometry_msgs::PoseStamped& target_pose) const;

  // utility operations (supports calculateDesiredGripperPose())
  void calculateDesiredGripperPosition();
  geometry_msgs::Vector3 getPositionDifference();
  geometry_msgs::Vector3 scalePositionDifference(const geometry_msgs::Vector3& position_difference);
  void applyPositionDifference(const geometry_msgs::Vector3& scaled_position_difference);

  void calculateDesiredGripperOrientation();
  RPY getOrientationDifference();
  RPY scaleOrientationDifference(const RPY& orientation_difference);
  void applyOrientationDifference(const RPY& scaled_orientation_difference);

  RPY getRPYFromPose(const geometry_msgs::PoseStamped& pose);
  double restrictAngleWithinPI(const double& angle);

  // callbacks
  void leftControllerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftControllerJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void toggleGripperStatus();

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  ros::Publisher target_frame_publisher_; // for teleoperateToPose() usage
  ros::Publisher gripper_pose_publisher_; // for pure teleoperation & shared control
  ros::Publisher gripper_status_publisher_;
  ros::Publisher record_signal_publisher_;
  ros::Publisher mobile_platform_velocity_publisher_;
  ros::Subscriber left_controller_pose_subscriber_;
  ros::Subscriber left_controller_joy_subscriber_;
  Tf2ListenerWrapper tf2_listener_;

  // ROS messages
  geometry_msgs::PoseStamped initial_left_controller_pose_;
  geometry_msgs::PoseStamped current_left_controller_pose_;
  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped current_gripper_pose_; // updated by lookupCurrentGripperPose()
  geometry_msgs::PoseStamped desired_gripper_pose_;
  geometry_msgs::Twist mobile_platform_velocity_;
  std_msgs::Bool desired_gripper_status_;
  std_msgs::Bool record_signal_;

  // flags
  bool is_begin_teleoperation_;  // true if teleoperation has not started yet
  bool is_position_change_enabled_;
  bool is_orientation_change_enabled_;
  GripperDirection gripper_direction_;

  // parameters
  double position_scale_;
  double orientation_scale_;
  double linear_velocity_scale_;
  double angular_velocity_scale_;
  bool use_shared_controller_;  // false if using pure teleoperation
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H