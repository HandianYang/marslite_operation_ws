#include "shared_control/shared_control.h"

#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

SharedControl::SharedControl(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(10)), begin_recording_(false), 
      intent_inference_(0.1) {
  this->initializePublishers();
  this->initializeSubscribers();
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void SharedControl::run() {
  while (nh_.ok()) {
    intent_inference_.updatePositionToBaseLink();
    intent_inference_.updateGripperMotionState();
    intent_inference_.updateBelief();
    
    visualization_msgs::MarkerArray belief_visualization = intent_inference_.getBeliefVisualization();
    belief_visualization_publisher_.publish(belief_visualization);
    
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
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/gripper/cmd_gripper", 1
  );
  belief_visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/marslite_control/intent_belief_marker", 1
  );
  gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/marslite_control/gripper_pose", 1
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