#ifndef MARSLITE_CONTROL_UTILS_RPY_H
#define MARSLITE_CONTROL_UTILS_RPY_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct RPY {
  double roll;
  double pitch;
  double yaw;

  void convertFromPoseStamped(const geometry_msgs::PoseStamped& pose_stamped) {
    return convertFromQuaternion(pose_stamped.pose.orientation);
  }

  void convertFromPose(const geometry_msgs::Pose& pose) {
    return convertFromQuaternion(pose.orientation);
  }

  void convertFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion q_tf2(q.x, q.y, q.z, q.w);
    if (q_tf2.length2() == 0) {
      roll = pitch = yaw = 0;
      return;
    }
    q_tf2.normalize();

    tf2::Matrix3x3 m_tf2(q_tf2);
    m_tf2.getRPY(roll, pitch, yaw);
  }

  geometry_msgs::Quaternion convertToQuaternion() {
    const tf2::Quaternion q_tf2 = convertToTF2Quaternion();
    geometry_msgs::Quaternion q;
    q.x = q_tf2.x();
    q.y = q_tf2.y();
    q.z = q_tf2.z();
    q.w = q_tf2.w();
    return q;
  }

  tf2::Quaternion convertToTF2Quaternion() {
    tf2::Quaternion q_tf2;
    q_tf2.setRPY(roll, pitch, yaw);
    q_tf2.normalize();
    return q_tf2;
  }
};

#endif // #ifndef MARSLITE_CONTROL_UTILS_RPY_H