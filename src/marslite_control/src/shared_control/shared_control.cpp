#include "shared_control/shared_control.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

SharedControl::SharedControl(const ros::NodeHandle& nh)
    : nh_(nh) {
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

void SharedControl::initializePublishers() {
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 1);
}

void SharedControl::initializeSubscribers() {
  user_desired_gripper_pose_subscriber_ = nh_.subscribe(
      "/cartesian_control/user_desired_gripper_pose", 1,
      &SharedControl::userDesiredGripperPoseCallback, this);
  user_desired_gripper_status_subscriber_ = nh_.subscribe(
      "/cartesian_control/user_desired_gripper_status", 1,
      &SharedControl::userDesiredGripperStatusCallback, this);
}

void SharedControl::parseParameters() {
  ros::NodeHandle pnh("~");
  // pnh.param("use_shared_controller", use_shared_controller_, false);

  // ROS_INFO_STREAM("  use_shared_controller: "
  //     << (use_shared_controller_ ? "true" : "false"));
}