#ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H
#define MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H

#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "detection_msgs/DetectedObjectArray.h"
#include "shared_control/intent_inference.h"
#include "utils/velocity_estimator.h"
#include "utils/tf2_listener_wrapper.h"
#include "utils/rpy.h"

class SharedControl {
 public:
  static inline constexpr double kDistanceTolerance = 1e-3;
  // normally >= 1.0 to enhance attractive force
  static inline constexpr double kAttractiveForceGain = 1.8;
  // "weak" means weak repulsive force (less restriction)
  static inline constexpr double kRepulsiveForceWeakGain = 0.8;
  // "strong" means strong repulsive force (more restriction)
  static inline constexpr double kRepulsiveForceStrongGain = 0.2;
  // [m] junction between "strong" gain and "weak" gain
  static inline constexpr double kRepulsiveForceJunctionDistance = 0.3;

  explicit SharedControl(const ros::NodeHandle& nh = ros::NodeHandle());

  bool callResetPoseService();

  void runInference();

  void runInferenceOnce();

 private:
  void parseParameters();
  void initializePublishers();
  void initializeSubscribers();
  void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& objects);
  void recordSignalCallback(const std_msgs::Bool::ConstPtr& signal);
  void positionSafetyButtonSignalCallback(const std_msgs::Bool::ConstPtr& signal);
  void orientationSafetyButtonSignalCallback(const std_msgs::Bool::ConstPtr& signal);
  void currentGripperPoseCallback(const geometry_msgs::PoseStamped& pose);
  void userDesiredGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& user_desired_gripper_pose);
  void userDesiredGripperStatusCallback(const std_msgs::Bool::ConstPtr& user_desired_gripper_status);
  void userCommandVelocityCallback(const geometry_msgs::Vector3::ConstPtr& user_command_velocity);

  void publishBlendedGripperPose();
  geometry_msgs::PoseStamped getBlendedPose();
  geometry_msgs::Point getBlendedPosition();
  geometry_msgs::Quaternion getBlendedOrientation();

  void publishIntentBeliefVisualization();
  
  void publishRobotState();
  
  void publishObjectsWithBelief();

  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  ros::Publisher belief_visualization_publisher_;
  ros::Publisher robot_state_publisher_;
  ros::Publisher objects_with_belief_publisher_;
  ros::ServiceClient reset_client_;
  ros::Subscriber detected_objects_subscriber_;
  ros::Subscriber record_signal_subscriber_;
  ros::Subscriber position_safety_button_signal_subscriber_;
  ros::Subscriber orientation_safety_button_signal_subscriber_;
  ros::Subscriber current_gripper_pose_subscriber_;
  ros::Subscriber user_desired_gripper_pose_subscriber_;
  ros::Subscriber user_desired_gripper_status_subscriber_;
  ros::Subscriber user_command_velocity_subscriber_;

  // self-defined class instances
  IntentInference intent_inference_;

  // ROS messages
  geometry_msgs::PoseStamped current_gripper_pose_;
  geometry_msgs::PoseStamped user_desired_gripper_pose_;
  std_msgs::Bool user_desired_gripper_status_;
  std_msgs::Bool position_safety_button_signal_;
  std_msgs::Bool orientation_safety_button_signal_;
  geometry_msgs::Vector3 user_command_velocity_;

  // true if running in simulation
  bool use_sim_;  
  // true if shared control is enabled (not affect the use of `recorded_objects`)
  bool shared_control_enabled_;
  // true if record_signal is received
  bool begin_recording_;
};

#endif // #ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H