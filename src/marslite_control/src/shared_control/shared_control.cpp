#include "shared_control/shared_control.h"

#include <visualization_msgs/MarkerArray.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

SharedControl::SharedControl(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(10)), begin_recording_(false), 
      intent_inference_(0.1) {
  this->initializePublishers();
  this->initializeSubscribers();
  this->parseParameters();
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void SharedControl::run() {
  static int count = 0;
  while (nh_.ok()) {
    geometry_msgs::Point user_direction;
    user_direction.x = user_desired_gripper_pose_.pose.position.x - last_user_desired_gripper_pose_.pose.position.x;
    user_direction.y = user_desired_gripper_pose_.pose.position.y - last_user_desired_gripper_pose_.pose.position.y;
    user_direction.z = user_desired_gripper_pose_.pose.position.z - last_user_desired_gripper_pose_.pose.position.z;
    intent_inference_.setGripperPosition(user_desired_gripper_pose_.pose.position);
    intent_inference_.setGripperDirection(user_direction);
    if (intent_inference_.updateBelief()) {
      detection_msgs::DetectedObject most_likely_goal = intent_inference_.getMostLikelyGoal();
      visualization_msgs::MarkerArray belief_visualization = intent_inference_.getBeliefVisualization();
      belief_visualization_publisher_.publish(belief_visualization);
    }
    
    rate_.sleep();
    ros::spinOnce();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void SharedControl::initializePublishers() {
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/cartesian_control/target_frame", 1
  );
  belief_visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/marslite_control/intent_belief_marker", 1
  );
}

void SharedControl::initializeSubscribers() {
  detected_objects_subscriber_ = nh_.subscribe(
      "/yolo/detected_objects", 1,
      &SharedControl::detectedObjectsCallback, this
  );
  record_signal_subscriber_ = nh_.subscribe(
      "/marslite_control/record_signal", 1,
      &SharedControl::recordSignalCallback, this
  );
  user_desired_gripper_pose_subscriber_ = nh_.subscribe(
      "/marslite_control/user_desired_gripper_pose", 1,
      &SharedControl::userDesiredGripperPoseCallback, this
  );
  user_desired_gripper_status_subscriber_ = nh_.subscribe(
      "/marslite_control/user_desired_gripper_status", 1,
      &SharedControl::userDesiredGripperStatusCallback, this
  );
}

void SharedControl::parseParameters() {
  ros::NodeHandle pnh("~");
  // pnh.param("use_shared_controller", use_shared_controller_, false);

  // ROS_INFO_STREAM("  use_shared_controller: "
  //     << (use_shared_controller_ ? "true" : "false"));
}