#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <mutex>

#include "cartesian_control/kinematics_constants.h"
#include "utils/rpy.h"
#include "utils/tf2_listener_wrapper.h"
#include "utils/velocity_estimator.h"

enum class RobotOperatingDirection : uint8_t {
  FRONT = 0,
  LEFT = 1,
  RIGHT = 2
};

class MotionControllerTeleoperation {
 public:
  // threshold for considering the trigger button of motion controllers is pressed
  static inline constexpr double kTriggerThreshold = 0.95;
  // tolerance for considering two positions are the same
  static inline constexpr double kPositionTolerance = 1e-3;
  // tolerance for considering two orientations are the same (in terms of quaternion)
  static inline constexpr double kOrientationTolerance = 1e-3;
  // buffer size for velocity estimator
  static inline constexpr size_t kEstimatorBufferSize = 10;
  // minimum speed for velocity estimator
  static inline constexpr double kEstimatorMinSpeed = 1e-2;

  explicit MotionControllerTeleoperation(const ros::NodeHandle& nh = ros::NodeHandle());

  inline void resetToFrontPose() {
    this->teleoperateToPose(kFrontInitialGripperPose);
    this->setRobotOperatingDirection(RobotOperatingDirection::FRONT);
  }

  inline void resetToLeftPose() {
    this->teleoperateToPose(kLeftInitialGripperPose);
    this->setRobotOperatingDirection(RobotOperatingDirection::LEFT);
  }

  inline void resetToRightPose() {
    this->teleoperateToPose(kRightInitialGripperPose);
    this->setRobotOperatingDirection(RobotOperatingDirection::RIGHT);
  }

  void teleoperateToPose(const geometry_msgs::PoseStamped& target_pose);

  inline void setRobotOperatingDirection(
      const RobotOperatingDirection& direction = RobotOperatingDirection::FRONT) {
    robot_operating_direction_ = direction;
  }

  void run();

 private:
  void parseParameters();
  void initializePublishers();
  void initializeSubscribers();

  inline void initializeGripperPose() {
    this->resetPositionalMovement();
    this->resetOrientationalMovement();
    initial_gripper_pose_ = desired_gripper_pose_;
  }

  inline void initializeLeftControllerPose() {
    initial_left_controller_pose_ = current_left_controller_pose_;
  }

  const bool targetPoseIsReached(const geometry_msgs::PoseStamped& target_pose) const;

  inline const bool isAnySafetyButtonPressed() const {
    return is_position_change_enabled_ || is_orientation_change_enabled_;
  }

  void calculateDesiredGripperPose();
  void calculateDesiredGripperPosition();
  geometry_msgs::Vector3 getPositionDifference();
  geometry_msgs::Vector3 scalePositionDifference(const geometry_msgs::Vector3& position_difference);
  void applyPositionDifference(const geometry_msgs::Vector3& scaled_position_difference);
  void calculateDesiredGripperOrientation();
  RPY getOrientationDifference();
  RPY scaleOrientationDifference(const RPY& orientation_difference);
  void applyOrientationDifference(const RPY& scaled_orientation_difference);

  void calculateUserCommandVelocity();

  inline void resetPositionalMovement() {
    // Shift a little to avoid immediate stop
    gripper_velocity_estimator_.estimateVelocity();
    gripper_velocity_ = gripper_velocity_estimator_.getEstimatedVelocity();
    desired_gripper_pose_.pose.position.x = current_gripper_pose_.pose.position.x + gripper_velocity_.x * 0.01;
    desired_gripper_pose_.pose.position.y = current_gripper_pose_.pose.position.y + gripper_velocity_.y * 0.01;
    desired_gripper_pose_.pose.position.z = current_gripper_pose_.pose.position.z + gripper_velocity_.z * 0.01;

    // reset user_command_velocity_
    user_command_velocity_estimator_.clear();
    user_command_velocity_ = geometry_msgs::Vector3();
  }

  inline void resetOrientationalMovement() {
    desired_gripper_pose_.pose.orientation = current_gripper_pose_.pose.orientation;
  }

  inline void publishDesiredGripperPose() {
    gripper_pose_publisher_.publish(desired_gripper_pose_);
  }

  inline void publishUserCommandVelocity() {
    user_command_velocity_publisher_.publish(user_command_velocity_);
  }

  void publishUserCommandVelocityMarker();

  inline void publisherGripperVelocity() {
    gripper_velocity_publisher_.publish(gripper_velocity_);
  }

  void publishGripperVelocityMarker();

  inline void publishMobilePlatformVelocity() {
    mobile_platform_velocity_publisher_.publish(mobile_platform_velocity_);
  }

  void currentGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftControllerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftControllerJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void toggleGripperStatus();
  void resetPoseSignalCallback(const std_msgs::Bool::ConstPtr& msg);

  static inline double restrictAngleWithinPI(const double& angle) {
    if (angle > M_PI) return angle - 2 * M_PI;
    if (angle < -M_PI) return angle + 2 * M_PI;
    return angle;
  }

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  ros::Publisher target_frame_publisher_; // for teleoperateToPose() usage
  ros::Publisher gripper_pose_publisher_; // for pure teleoperation & shared control
  ros::Publisher gripper_status_publisher_;
  ros::Publisher record_signal_publisher_;
  ros::Publisher position_safety_button_signal_publisher_;
  ros::Publisher orientation_safety_button_signal_publisher_;
  ros::Publisher mobile_platform_velocity_publisher_;

  ros::Publisher user_command_velocity_publisher_;
  ros::Publisher user_command_velocity_marker_publisher_;
  ros::Publisher gripper_velocity_publisher_;
  ros::Publisher gripper_velocity_marker_publisher_;
  
  ros::Subscriber current_gripper_pose_subscriber_;
  ros::Subscriber left_controller_pose_subscriber_;
  ros::Subscriber left_controller_joy_subscriber_;
  ros::Subscriber reset_pose_signal_subscriber_;

  // ROS messages
  geometry_msgs::PoseStamped initial_left_controller_pose_;
  geometry_msgs::PoseStamped current_left_controller_pose_;
  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped current_gripper_pose_; // updated by lookupCurrentGripperPose()
  geometry_msgs::PoseStamped desired_gripper_pose_;
  geometry_msgs::Vector3 user_command_velocity_;
  geometry_msgs::Vector3 gripper_velocity_;
  geometry_msgs::Twist mobile_platform_velocity_;
  std_msgs::Bool desired_gripper_status_;
  std_msgs::Bool record_signal_;
  std_msgs::Bool position_safety_button_signal_;
  std_msgs::Bool orientation_safety_button_signal_;

  // flags
  bool is_begin_teleoperation_;  // true if teleoperation has not started yet
  bool is_position_change_enabled_;
  bool is_orientation_change_enabled_;
  bool use_shared_controller_;  // false if using pure teleoperation

  // parameters
  double position_scale_;
  double orientation_scale_;
  double linear_velocity_scale_;
  double angular_velocity_scale_;

  std::mutex mutex_;
  RobotOperatingDirection robot_operating_direction_;
  VelocityEstimator user_command_velocity_estimator_;
  VelocityEstimator gripper_velocity_estimator_;
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H