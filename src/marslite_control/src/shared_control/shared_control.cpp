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
    // if (!begin_shared_control_) {
    //   rate_.sleep();
    //   ros::spinOnce();
    //   continue;
    // }

    intent_inference_.updateBelief();
    this->publishIntentBeliefVisualization();
    this->publishBlendingGripperPose();
    
    rate_.sleep();
    ros::spinOnce();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void SharedControl::initializePublishers() {
  current_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/marslite_control/current_gripper_pose", 1
  );
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/cartesian_control/target_frame", 1
  );
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/gripper/cmd_gripper", 1
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

void SharedControl::publishIntentBeliefVisualization() {
  visualization_msgs::MarkerArray belief_visualization
      = intent_inference_.getBeliefVisualization();
  belief_visualization_publisher_.publish(belief_visualization);
}

void SharedControl::publishBlendingGripperPose() {
  geometry_msgs::PoseStamped target_pose = this->getTargetPose();
  desired_gripper_pose_publisher_.publish(target_pose);
  // TODO: Test blending result
  // current_gripper_transform_ = tf2_listener_.lookupTransform("base_link", "tm_gripper");
  // geometry_msgs::Point blended_direction = this->getBlendingDirection();
  // geometry_msgs::PoseStamped desired_gripper_pose;
  // desired_gripper_pose.header = current_gripper_transform_.header;
  // desired_gripper_pose.pose.position.x = current_gripper_transform_.transform.translation.x + blended_direction.x;
  // desired_gripper_pose.pose.position.y = current_gripper_transform_.transform.translation.y + blended_direction.y;
  // desired_gripper_pose.pose.position.z = current_gripper_transform_.transform.translation.z + blended_direction.z;
  // desired_gripper_pose.pose.orientation = current_gripper_transform_.transform.rotation;
  
  // TODO: 
  // desired_gripper_pose_publisher_.publish(desired_gripper_pose);   // shared control
}

// geometry_msgs::Point SharedControl::getBlendingDirection() {
//   const double alpha = intent_inference_.getAlpha();
//   geometry_msgs::Point user_cmd =
//       this->normalizeDirection(controller_direction_estimator_.getAveragedDirection());
  
//   detection_msgs::DetectedObject most_likely_goal = intent_inference_.getMostLikelyGoal();
//   geometry_msgs::Point robot_cmd = 
//       this->normalizeDirection(intent_inference_.getGoalDirection(most_likely_goal));

//   geometry_msgs::Point blended_cmd;
//   blended_cmd.x = (1 - alpha) * user_cmd.x + alpha * robot_cmd.x;
//   blended_cmd.y = (1 - alpha) * user_cmd.y + alpha * robot_cmd.y;
//   blended_cmd.z = (1 - alpha) * user_cmd.z + alpha * robot_cmd.z;

//   const double kFactor = 0.1; // [m/s]
//   geometry_msgs::Point scaled_blended_cmd = this->scaleDirection(blended_cmd, kFactor);
//   return blended_cmd;
// }

// geometry_msgs::Point SharedControl::normalizeDirection(const geometry_msgs::Point& direction) {
//   geometry_msgs::Point normalized_direction;
//   const double norm = std::sqrt(
//       direction.x * direction.x +
//       direction.y * direction.y +
//       direction.z * direction.z
//   );

//   if (norm < 1e-6) {
//     normalized_direction.x = normalized_direction.y = normalized_direction.z = 0.0;
//   } else {
//     normalized_direction.x = direction.x / norm;
//     normalized_direction.y = direction.y / norm;
//     normalized_direction.z = direction.z / norm;
//   }
//   return normalized_direction;
// }

// geometry_msgs::Point SharedControl::scaleDirection(
//     const geometry_msgs::Point& direction, const double& scale) {
//   const geometry_msgs::Point normalized_direction = this->normalizeDirection(direction);
//   geometry_msgs::Point scaled_direction;
//   scaled_direction.x = normalized_direction.x * scale;
//   scaled_direction.y = normalized_direction.y * scale;
//   scaled_direction.z = normalized_direction.z * scale;
//   return scaled_direction;
// }

geometry_msgs::PoseStamped SharedControl::getTargetPose() {
  const double confidence = intent_inference_.getConfidence();
  if (!(intent_inference_.isApproaching() && confidence > kLockTargetConfidenceThreshold)) {
    // TODO: Modify the confidence threshold
    return user_desired_gripper_pose_;
  }  
    
  geometry_msgs::Point most_likely_goal_position
      = intent_inference_.getMostLikelyGoalPosition();
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position = most_likely_goal_position;
  target_pose.pose.orientation = current_gripper_pose_.pose.orientation;
  return target_pose;
}

void SharedControl::detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& objects) {
  if (begin_recording_) {
    intent_inference_.setRecordedObjects(*objects);
    ROS_INFO_STREAM("Received " << objects->objects.size() << " detected objects.");
    
    begin_recording_ = false;
    begin_shared_control_ = (objects->objects.size() > 0);
  }
}

void SharedControl::recordSignalCallback(const std_msgs::Bool::ConstPtr& signal) {
  begin_recording_ = true;
}

void SharedControl::userDesiredGripperPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  user_desired_gripper_pose_ = *msg;
  
  this->lookupCurrentGripperPose();
  intent_inference_.setGripperPosition(current_gripper_pose_.pose.position);

  controller_direction_estimator_.addwaypoint(user_desired_gripper_pose_.pose.position);
  geometry_msgs::Point user_desired_gripper_direction
      = controller_direction_estimator_.getAveragedDirection();
  intent_inference_.setGripperDirection(user_desired_gripper_direction);

  // TODO: Uncomment this line while testing shared control
  // desired_gripper_pose_publisher_.publish(*user_desired_gripper_pose);
}

void SharedControl::userDesiredGripperStatusCallback(
    const std_msgs::Bool::ConstPtr& user_desired_gripper_status) {
  // Directly publish the user's desired gripper status becuase we don't 
  //   need to process it through shared control.
  desired_gripper_status_publisher_.publish(*user_desired_gripper_status);
  // Also update gripper_motion_state_ in intent_inference_
  intent_inference_.setGripperStatus(*user_desired_gripper_status);
}