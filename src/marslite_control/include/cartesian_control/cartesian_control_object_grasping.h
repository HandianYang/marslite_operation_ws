#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_OBJECT_GRASPING_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_OBJECT_GRASPING_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include "detection_msgs/DetectedObject.h"
#include "detection_msgs/DetectedObjectArray.h"
#include "detection_msgs/DriveToInitialPoint.h"
#include "detection_msgs/DriveToObservePoint.h"
#include "detection_msgs/TriggerRecording.h"
#include "detection_msgs/ObjectGrasping.h"

#include <algorithm>
#include <vector>
#include <unordered_map>

const static int kFrameToRecord = 10;

const geometry_msgs::PoseStamped kInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.712;
  pose_stamped.pose.position.y = -0.122;
  pose_stamped.pose.position.z = 1.043;
  pose_stamped.pose.orientation.x = 0.5;
  pose_stamped.pose.orientation.y = 0.5;
  pose_stamped.pose.orientation.z = 0.5;
  pose_stamped.pose.orientation.w = 0.5;
  return pose_stamped;
}();

const geometry_msgs::PoseStamped kObservingGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.621;
  pose_stamped.pose.position.y = -0.129;
  pose_stamped.pose.position.z = 0.912;
  pose_stamped.pose.orientation.x = 0.616;
  pose_stamped.pose.orientation.y = 0.583;
  pose_stamped.pose.orientation.z = 0.369;
  pose_stamped.pose.orientation.w = 0.380;
  return pose_stamped;
}();

class CartesianControlObjectGrasping {
public:
  explicit CartesianControlObjectGrasping(const ros::NodeHandle& nh = ros::NodeHandle());
  void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher target_frame_pub_;
  ros::Publisher gripper_pub_;
  ros::Subscriber detected_objects_sub_;
  ros::ServiceServer drive_to_initial_point_service_;
  ros::ServiceServer drive_to_observe_point_service_;
  ros::ServiceServer trigger_recording_service_;
  ros::ServiceServer object_grasping_service_;

  detection_msgs::DetectedObjectArray detected_objects_;
  geometry_msgs::PoseStamped target_gripper_pose_;
  std_msgs::Bool gripper_;

  std::unordered_map<int, std::vector<geometry_msgs::Point>> record_buffer_;
  std::unordered_map<int, geometry_msgs::Point> recorded_object_positions_;

  // flags
  bool has_pending_record_;

private:
  // service trigger functions
  bool driveToInitialPoint(detection_msgs::DriveToInitialPoint::Request& req,
                           detection_msgs::DriveToInitialPoint::Response& res);
  bool driveToObservePoint(detection_msgs::DriveToObservePoint::Request& req,
                           detection_msgs::DriveToObservePoint::Response& res);
  bool recordInitialPosition(detection_msgs::TriggerRecording::Request& req,
                             detection_msgs::TriggerRecording::Response& res);
  bool graspObjectWithId(detection_msgs::ObjectGrasping::Request& req,
                         detection_msgs::ObjectGrasping::Response& res);

  void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& msg);
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_OBJECT_GRASPING_H