#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_OBJECT_GRASPING_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_OBJECT_GRASPING_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "detection_msgs/DetectedObjectArray.h"


class ObjectGraspingWrapper {
public:
  explicit ObjectGraspingWrapper(const ros::NodeHandle& nh = ros::NodeHandle());
  
  inline detection_msgs::DetectedObjectArray getDetectedObjects() const {
    return detected_objects_;
  }

  inline void publishTargetFrame(const geometry_msgs::PoseStamped& target_pose) {
    target_frame_publisher_.publish(target_pose);
  }

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher target_frame_publisher_;
  ros::Subscriber detected_objects_subscriber_;
  detection_msgs::DetectedObjectArray detected_objects_;

private:
  // callbacks
  inline void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& msg) {
    detected_objects_ = *msg;
  }
};

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_OBJECT_GRASPING_H