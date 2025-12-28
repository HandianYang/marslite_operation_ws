#ifndef MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H
#define MARSLITE_CONTROL_UTILS_TF2_LISTENER_WRAPPER_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Tf2ListenerWrapper {
 public:
  Tf2ListenerWrapper(): tf_buffer_(), tf_listener_(tf_buffer_) {}
  
  /**
   * @brief Lookup the transform between two frames and return it as type `T`.
   * @tparam T The type of the transform to return. Supported types are:
   * 
   * - `geometry_msgs::TransformStamped`
   * 
   * - `geometry_msgs::PoseStamped`
   * 
   * - `geometry_msgs::Pose`
   * 
   * - `geometry_msgs::PointStamped`
   * 
   * - `geometry_msgs::Point`
   * 
   * @param target_frame The target frame ID.
   * @param source_frame The source frame ID.
   * @return The transform from source_frame to target_frame as type `T`.
   */
  template <typename T>
  T lookupTransform(const std::string& target_frame, const std::string& source_frame);

  /**
   * @brief Transform `input_data` from its original frame to `target_frame`
   * @tparam T The type of the data to be transformed. Must be compatible with tf2::doTransform.
   * @param input_data The data to be transformed. Must have a `header` member with `frame_id` field.
   * @param target_frame The frame to which the data should be transformed.
   * @return The transformed data. If transformation fails, returns a default-constructed `T`.
   */
  template <typename T>
  T transformData(const T& input_data, const std::string& target_frame) {
    T output_data;
    try {
      geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
          target_frame, 
          input_data.header.frame_id, 
          ros::Time(0),
          ros::Duration(1.0)
      );
      tf2::doTransform(input_data, output_data, transform_stamped);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Transform failure: %s", ex.what());
      return T(); 
    }

    return output_data;
  }
    
 private:
  /**
   * @brief Helper function to get the raw TransformStamped between two frames.
   * @param target_frame The target frame ID.
   * @param source_frame The source frame ID.
   * @return The TransformStamped between the frames. If lookup fails, returns an empty TransformStamped.
   */
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

/// Specialization for geometry_msgs::TransformStamped
template <>
inline geometry_msgs::TransformStamped Tf2ListenerWrapper::lookupTransform<geometry_msgs::TransformStamped>(
    const std::string& target_frame, const std::string& source_frame) {
  return getRawTransform(target_frame, source_frame);
}

/// Specialization for geometry_msgs::PoseStamped
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

/// Specialization for geometry_msgs::Pose
template<>
inline geometry_msgs::Pose Tf2ListenerWrapper::lookupTransform<geometry_msgs::Pose>(
    const std::string& target_frame, const std::string& source_frame) {
  geometry_msgs::TransformStamped ts = getRawTransform(target_frame, source_frame);
  geometry_msgs::Pose p;
  p.position.x = ts.transform.translation.x;
  p.position.y = ts.transform.translation.y;
  p.position.z = ts.transform.translation.z;
  p.orientation = ts.transform.rotation;
  return p;
}

/// Specialization for geometry_msgs::PointStamped
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

/// Specialization for geometry_msgs::Point
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