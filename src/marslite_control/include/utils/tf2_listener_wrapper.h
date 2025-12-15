#ifndef MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H
#define MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Tf2ListenerWrapper {
 public:
  Tf2ListenerWrapper(): tf_buffer_(), tf_listener_(tf_buffer_) {}
  
  template <typename T>
  T lookupTransform(const std::string& target_frame, const std::string& source_frame);
    
 private:
  geometry_msgs::TransformStamped getRawTransform(
      const std::string& target_frame,
      const std::string& source_frame) {
    try {
      return tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s Return empty transform.", ex.what());
      return geometry_msgs::TransformStamped();
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

template <>
inline geometry_msgs::TransformStamped Tf2ListenerWrapper::lookupTransform<geometry_msgs::TransformStamped>(
    const std::string& target_frame, const std::string& source_frame) {
  return getRawTransform(target_frame, source_frame);
}

template <>
inline geometry_msgs::PoseStamped Tf2ListenerWrapper::lookupTransform<geometry_msgs::PoseStamped>(
    const std::string& target_frame, const std::string& source_frame) {
  geometry_msgs::TransformStamped ts = getRawTransform(target_frame, source_frame);
  geometry_msgs::PoseStamped ps;
  ps.header = ts.header;
  ps.pose.position.x = ts.transform.translation.x;
  ps.pose.position.y = ts.transform.translation.y;
  ps.pose.position.z = ts.transform.translation.z;
  ps.pose.orientation = ts.transform.rotation;
  return ps;
}

template <>
inline geometry_msgs::PointStamped Tf2ListenerWrapper::lookupTransform<geometry_msgs::PointStamped>(
    const std::string& target_frame, const std::string& source_frame) {
  geometry_msgs::TransformStamped ts = getRawTransform(target_frame, source_frame);
  geometry_msgs::PointStamped pts;
  pts.header = ts.header;
  pts.point.x = ts.transform.translation.x;
  pts.point.y = ts.transform.translation.y;
  pts.point.z = ts.transform.translation.z;
  return pts;
}

template <>
inline geometry_msgs::Point Tf2ListenerWrapper::lookupTransform<geometry_msgs::Point>(
    const std::string& target_frame, const std::string& source_frame) {
  geometry_msgs::TransformStamped ts = getRawTransform(target_frame, source_frame);
  geometry_msgs::Point pt;
  pt.x = ts.transform.translation.x;
  pt.y = ts.transform.translation.y;
  pt.z = ts.transform.translation.z;
  return pt;
}


#endif // #ifndef MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H