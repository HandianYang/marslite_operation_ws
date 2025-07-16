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
#include "detection_msgs/GraspObjectWithID.h"
#include "detection_msgs/RecordObjectPosition.h"


#include <algorithm>
#include <vector>
#include <unordered_map>

// const static int kFrameToRecord = 10;

// base_frame: /base_link
// end_effector_frame: /tm_gripper
const geometry_msgs::PoseStamped kInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.888;
  pose_stamped.pose.position.y = -0.122;
  pose_stamped.pose.position.z = 1.035;
  pose_stamped.pose.orientation.x = 0.5;
  pose_stamped.pose.orientation.y = 0.5;
  pose_stamped.pose.orientation.z = 0.5;
  pose_stamped.pose.orientation.w = 0.5;
  return pose_stamped;
}();

// observing point for shelves
//   base_frame: /base_link
//   end_effector_frame: /tm_gripper
const geometry_msgs::PoseStamped kObservingGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.872;
  pose_stamped.pose.position.y = -0.123;
  pose_stamped.pose.position.z = 0.969;
  pose_stamped.pose.orientation.x = 0.578;
  pose_stamped.pose.orientation.y = 0.577;
  pose_stamped.pose.orientation.z = 0.408;
  pose_stamped.pose.orientation.w = 0.408;
  return pose_stamped;
}();

class CartesianControlObjectGrasping {
public:
  explicit CartesianControlObjectGrasping(const ros::NodeHandle& nh = ros::NodeHandle());
  // void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher target_frame_pub_;
  ros::Publisher gripper_pub_;
  ros::Subscriber detected_objects_sub_;
  // TODO: Move out to test
  ros::ServiceServer drive_to_initial_point_service_;
  ros::ServiceServer drive_to_observe_point_service_;
  ros::ServiceServer record_object_position_service_;
  ros::ServiceServer grasp_object_with_id_service_;

  detection_msgs::DetectedObjectArray detected_objects_;
  detection_msgs::DetectedObjectArray recorded_objects_;
  geometry_msgs::PoseStamped target_gripper_pose_;
  std_msgs::Bool gripper_;

private:
  // service trigger functions
  // TODO: Move out to test
  inline bool driveToInitialPoint(
      detection_msgs::DriveToInitialPoint::Request& req,
      detection_msgs::DriveToInitialPoint::Response& res)
  {
    target_frame_pub_.publish(kInitialGripperPose);
    return true;
  }

  inline bool driveToObservePoint(
      detection_msgs::DriveToObservePoint::Request& req,
      detection_msgs::DriveToObservePoint::Response& res)
  {
    target_frame_pub_.publish(kObservingGripperPose);
    return true;
  }
  
  inline bool recordObjectPosition(
      detection_msgs::RecordObjectPosition::Request& req,
      detection_msgs::RecordObjectPosition::Response& res)
  {
    recorded_objects_ = detected_objects_;
    if (recorded_objects_.objects.empty()) {
      res.result = "No objects detected to record.";
      res.success = false;
      return false;
    }

    std::ostringstream oss;
    oss << recorded_objects_.objects.size() << " objects detected.\n";
    for (const auto& object : recorded_objects_.objects) {
      oss << "[" << std::fixed << std::setprecision(3)
          << object.centroid.x << ", "
          << object.centroid.y << ", "
          << object.centroid.z << "]\n";
    }
    res.result = oss.str();
    res.success = true;
    return true;
  }
            
  inline bool graspObjectWithId(
      detection_msgs::GraspObjectWithID::Request& req,
      detection_msgs::GraspObjectWithID::Response& res)
  {
    if (req.id < 0 || req.id >= static_cast<int>(recorded_objects_.objects.size())) {
      res.result = "Invalid object ID! Skip grasping.";
      res.success = false;
      return false;
    }

    target_gripper_pose_.header.frame_id = "base_link";
    target_gripper_pose_.pose.position = recorded_objects_.objects[req.id].centroid;
    target_gripper_pose_.pose.orientation = kInitialGripperPose.pose.orientation;
    target_frame_pub_.publish(target_gripper_pose_);
    res.result = "Grasping object with ID " + std::to_string(req.id);
    res.success = true;
    return true;
  }

  // callbacks
  inline void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& msg)
  {
    detected_objects_ = *msg;
  }
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_OBJECT_GRASPING_H