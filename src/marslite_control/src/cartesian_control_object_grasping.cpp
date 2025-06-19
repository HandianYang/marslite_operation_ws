#include "cartesian_control/cartesian_control_object_grasping.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

CartesianControlObjectGrasping::CartesianControlObjectGrasping(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(25)), has_pending_record_(false)
{
  target_frame_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 10);
  gripper_pub_ = nh_.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
  detected_objects_sub_ = nh_.subscribe("/yolo/detected_objects", 10, &CartesianControlObjectGrasping::detectedObjectsCallback, this);

  drive_to_initial_point_service_ = nh_.advertiseService(
      "/drive_to_initial_point", &CartesianControlObjectGrasping::driveToInitialPoint, this);
  drive_to_observe_point_service_ = nh_.advertiseService(
      "/drive_to_observe_point", &CartesianControlObjectGrasping::driveToObservePoint, this);
  trigger_recording_service_ = nh_.advertiseService(
      "/record_initial_position", &CartesianControlObjectGrasping::recordInitialPosition, this);
  object_grasping_service_ = nh_.advertiseService(
      "/grasp_object_with_id", &CartesianControlObjectGrasping::graspObjectWithId, this);
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void CartesianControlObjectGrasping::run()
{
  while (ros::ok()) {
    

    rate_.sleep();
    ros::spinOnce();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

bool CartesianControlObjectGrasping::driveToInitialPoint(
    detection_msgs::DriveToInitialPoint::Request& req,
    detection_msgs::DriveToInitialPoint::Response& res)
{
  target_frame_pub_.publish(kInitialGripperPose);
  return true;
}

bool CartesianControlObjectGrasping::driveToObservePoint(
    detection_msgs::DriveToObservePoint::Request& req,
    detection_msgs::DriveToObservePoint::Response& res)
{
  target_frame_pub_.publish(kObservingGripperPose);
  return true;
}

bool CartesianControlObjectGrasping::recordInitialPosition(
    detection_msgs::TriggerRecording::Request& req,
    detection_msgs::TriggerRecording::Response& res)
{
  ROS_INFO(" Recording initial positions started...");
  record_buffer_.clear();
  has_pending_record_ = true;
  res.success = true;
  return true;
}

bool CartesianControlObjectGrasping::graspObjectWithId(
    detection_msgs::ObjectGrasping::Request& req,
    detection_msgs::ObjectGrasping::Response& res)
{
  ROS_INFO("  Grasping object with id=%d...", req.id);
  target_gripper_pose_.header.frame_id = "base_link";
  target_gripper_pose_.pose.position = recorded_object_positions_[req.id];
  target_gripper_pose_.pose.orientation = kInitialGripperPose.pose.orientation;
  target_frame_pub_.publish(target_gripper_pose_);
  res.success = true;
  return true;
}

void CartesianControlObjectGrasping::detectedObjectsCallback(
    const detection_msgs::DetectedObjectArray::ConstPtr& msg)
{
  detected_objects_ = *msg;

  if (has_pending_record_) {
    for (const detection_msgs::DetectedObject& object : detected_objects_.objects) {
      if (record_buffer_.find(object.id) == record_buffer_.end()) {
        record_buffer_[object.id] = std::vector<geometry_msgs::Point>();
      }
      record_buffer_[object.id].push_back(object.position);
    }

    // Check if any object appears in more than 5 frames
    bool any_object_has_enough_frames = false;
    for (const auto& [id, positions] : record_buffer_) {
      if (positions.size() > kFrameToRecord) {
        any_object_has_enough_frames = true;
        break;
      }
    }

    if (any_object_has_enough_frames) {
      has_pending_record_ = false;
      for (const auto& [id, positions] : record_buffer_) {
        if (positions.size() < 5) {
          // ROS_WARN("Not enough positions recorded for object ID %d. Expected %d, got %zu.", id, kFrameToRecord, positions.size());
          continue;
        }
        recorded_object_positions_[id] = positions.back(); // Get the last recorded position
      }
      ROS_INFO(" Recording initial positions completed.");
    }
  }
}