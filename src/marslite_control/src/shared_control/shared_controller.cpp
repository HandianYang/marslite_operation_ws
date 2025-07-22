#include "shared_control/shared_controller.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

SharedController::SharedController(const ros::NodeHandle& nh)
    : nh_(nh), use_shared_controller_(false) {
  this->initializePublishers();
  this->initializeSubscribers();
  this->parseParameters();
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */


/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void SharedController::initializePublishers() {
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/target_frame", 1);
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/gripper/cmd_gripper", 1);
}

void SharedController::initializeSubscribers() {
  user_desired_gripper_pose_subscriber_ = nh_.subscribe(
      "/cartesian_control/user_desired_gripper_pose", 1,
      &SharedController::userDesiredGripperPoseCallback, this);
  user_desired_gripper_status_subscriber_ = nh_.subscribe(
      "/cartesian_control/user_desired_gripper_status", 1,
      &SharedController::userDesiredGripperStatusCallback, this);
}

void SharedController::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("use_shared_controller", use_shared_controller_, false);

  ROS_INFO_STREAM("  use_shared_controller: "
      << (use_shared_controller_ ? "true" : "false"));
}