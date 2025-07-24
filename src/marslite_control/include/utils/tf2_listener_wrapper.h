#ifndef MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H
#define MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Tf2ListenerWrapper {
public:
  Tf2ListenerWrapper(): tf_buffer_(), tf_listener_(tf_buffer_) {}
    
  inline geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame,
      const std::string& source_frame) {
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s. Return empty TransformStamped() instead.", ex.what());
    }
    return transform_stamped;
  }

  inline geometry_msgs::PoseStamped lookupTransformInPoseStamped(
      const std::string& target_frame,
      const std::string& source_frame)
  {
    geometry_msgs::TransformStamped transform_stamped
        = this->lookupTransform(target_frame, source_frame);
    
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = transform_stamped.header;
    pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
    pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
    pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
    pose_stamped.pose.orientation = transform_stamped.transform.rotation;
    return pose_stamped;
  } 
    
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};


#endif // #ifndef MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H