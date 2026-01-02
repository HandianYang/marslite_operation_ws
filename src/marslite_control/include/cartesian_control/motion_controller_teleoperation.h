#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H

#include <Eigen/Dense>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

#include "utils/hybrid_pose.h"
#include "utils/rpy.h"
#include "utils/tf2_listener_wrapper.h"
#include "utils/velocity_estimator.h"


class MotionControllerTeleoperation {
 public:
  static inline constexpr double kTriggerThreshold = 0.95;
  // deadzone for analog joystick to avoid unintentional small movements
  static inline constexpr double kAnalogJoystickDeadzone = 0.8;
  // [s] cooldown time for toggling gripper status
  static inline constexpr double kGripperToggleCooldownTime = 0.5;
  // tolerance for considering two positions are the same
  static inline constexpr double kPositionTolerance = 1e-3;
  // tolerance for considering two orientations are the same (in terms of quaternion)
  static inline constexpr double kOrientationTolerance = 0.01;
  // buffer size for velocity estimator
  static inline constexpr size_t kEstimatorBufferSize = 6;
  // minimum speed for velocity estimator
  static inline constexpr double kEstimatorMinSpeed = 1e-2;

  explicit MotionControllerTeleoperation(const ros::NodeHandle& nh = ros::NodeHandle());

  inline void resetToFrontPose() {
    this->teleoperateToPose(
        MotionControllerTeleoperation::generateInitialGripperPose(
            0.0, control_view_offset_, use_sim_
        )
    );
  }

  inline void resetToLeftPose() {
    this->teleoperateToPose(
        MotionControllerTeleoperation::generateInitialGripperPose(
            M_PI / 2, control_view_offset_, use_sim_
        )
    );
  }

  inline void resetToLeftFrontPose() {        
    this->teleoperateToPose(
        MotionControllerTeleoperation::generateInitialGripperPose(
            M_PI / 4, control_view_offset_, use_sim_
        )
    );
  }

  inline void resetToRightPose() {
    this->teleoperateToPose(
        MotionControllerTeleoperation::generateInitialGripperPose(
            -M_PI / 2, control_view_offset_, use_sim_
        )
    );
  }

  static geometry_msgs::PoseStamped generateInitialGripperPose(
      const double& control_view_angle = 0.0,
      const double& control_view_offset = 0.0,
      const bool& use_sim = false
  );

  void teleoperateToPose(const geometry_msgs::PoseStamped& target_pose);

  void run();

 private:
  struct CylindricalPose {
    double radius;
    double yaw;
    double height;
  };

  void parseParameters();
  void initializePublishers();
  void initializeSubscribers();
  void initializeHeadersForMessages();
  void initializeVelocityEstimators();
  void leftControllerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftControllerJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void toggleGripperStatus();
  void resetPoseSignalCallback(const std_msgs::Bool::ConstPtr& msg);
  bool resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  const bool targetPoseIsReached(const geometry_msgs::PoseStamped& target_pose) const;
  void initializeGripperPose();
  void initializeLeftControllerPose();
  void updateGripperPose();

  inline const bool isAnySafetyButtonPressed() const {
    return is_position_change_enabled_ || is_orientation_change_enabled_;
  }
  
  void calculateDesiredGripperPose();
  void calculateDesiredGripperOrientation();
  RPY getOrientationDifference();
  RPY scaleOrientationDifference(const RPY& orientation_difference);
  void applyOrientationDifference(const RPY& scaled_orientation_difference);
  void calculateDesiredGripperPosition();
  CylindricalPoint getCylindricalPositionDifference();
  CylindricalPoint scaleCylindricalPositionDifference(const CylindricalPoint& position_difference);
  void applyCylindricalPositionDifference(const CylindricalPoint& scaled_position_difference);
  void updateGripperVelocity();
  void calculateGripperVelocityMarker();
  void calculateUserCommandVelocity();
  void calculateUserCommandVelocityMarker();

  inline void publishDesiredGripperPose() {
    geometry_msgs::PoseStamped desired_pose;
    desired_pose.header = desired_gripper_hybrid_pose_.header;
    desired_pose.pose = desired_gripper_hybrid_pose_.toCartesianPose();
    gripper_pose_publisher_.publish(desired_pose);
  }

  inline void publisherGripperVelocity() {
    gripper_velocity_publisher_.publish(gripper_velocity_);
  }

  inline void publishGripperVelocityMarker() {
    gripper_velocity_marker_publisher_.publish(gripper_velocity_marker_);
  }

  inline void publishUserCommandVelocity() {
    user_command_velocity_publisher_.publish(user_command_velocity_);
  }

  inline void publishUserCommandVelocityMarker() {
    user_command_velocity_marker_publisher_.publish(user_command_velocity_marker_);
  }

  inline void publishMobilePlatformVelocity() {
    mobile_platform_velocity_publisher_.publish(mobile_platform_velocity_);
  }

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
  ros::ServiceServer reset_service_;
  ros::Subscriber left_controller_pose_subscriber_;
  ros::Subscriber left_controller_joy_subscriber_;
  ros::Subscriber reset_pose_signal_subscriber_;
  

  // ROS messages
  geometry_msgs::PointStamped shoulder_position_;
  geometry_msgs::Vector3 user_command_velocity_;
  geometry_msgs::Vector3 gripper_velocity_;
  geometry_msgs::Twist mobile_platform_velocity_;
  std_msgs::Bool desired_gripper_status_;
  std_msgs::Bool record_signal_;
  std_msgs::Bool position_safety_button_signal_;
  std_msgs::Bool orientation_safety_button_signal_;
  visualization_msgs::Marker user_command_velocity_marker_;
  visualization_msgs::Marker gripper_velocity_marker_;

  // flags
  bool is_position_change_enabled_;
  bool is_orientation_change_enabled_;
  bool use_shared_controller_;  // false if using pure teleoperation
  bool use_sim_;  // true if running in simulation
  bool is_shoulder_position_calibrated_;  // true if the initial shoulder point is calibrated

  // parameters
  double position_scale_;
  double orientation_scale_;
  double linear_platform_velocity_scale_;
  double angular_platform_velocity_scale_;
  double initial_lateral_offset_;
  double accumulated_radius_difference_;

  // [radian] angle difference between first joint yaw angle and control view direction
  double control_view_offset_;
  HybridPose initial_gripper_hybrid_pose_;
  HybridPose current_gripper_hybrid_pose_;
  HybridPose desired_gripper_hybrid_pose_;
  HybridPose initial_left_controller_hybrid_pose_;
  HybridPose current_left_controller_hybrid_pose_;
  HybridPose previous_left_controller_hybrid_pose_; // last frame of current_left_controller_hybrid_pose_

  std::mutex desired_gripper_pose_mutex_;  // protect desired_gripper_pose_
  VelocityEstimator user_command_velocity_estimator_;
  VelocityEstimator gripper_velocity_estimator_;
  Tf2ListenerWrapper tf2_listener_;
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_MOTION_CONTROLLER_TELEOPERATION_H