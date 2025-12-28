#ifndef MARSLITE_CONTROL_UTILS_CYLINDRICAL_POSE_H
#define MARSLITE_CONTROL_UTILS_CYLINDRICAL_POSE_H

#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include "utils/rpy.h"

struct CylindricalPoint {
  double radius = 0.0;
  double yaw = 0.0;
  double height = 0.0;
};

struct CylindricalPointStamped {
  std_msgs::Header header;
  CylindricalPoint position;
};

struct HybridPose {
  std_msgs::Header header;
  geometry_msgs::Point cartesian_position;
  CylindricalPoint cylindrical_position;
  geometry_msgs::Quaternion orientation;

  void updateFromCartesianPose(const geometry_msgs::PoseStamped& cartesian_pose,
                               const double& control_view_offset = 0.0) {
    // Update Cartesian pose
    header = cartesian_pose.header;
    cartesian_position = cartesian_pose.pose.position;
    orientation = cartesian_pose.pose.orientation;

    // Update Cylindrical pose
    RPY pose_rpy; pose_rpy.convertFromPoseStamped(cartesian_pose);
    cylindrical_position.yaw = pose_rpy.yaw;
    cylindrical_position.height = cartesian_pose.pose.position.z;
    const double control_view_angle = pose_rpy.yaw - control_view_offset;
    cylindrical_position.radius =
        cartesian_position.x * std::cos(control_view_angle) +
        cartesian_position.y * std::sin(control_view_angle);
  }

  void updateFromCylindricalPoint(const CylindricalPoint& cylindrical_position,
                                  const double& control_view_offset = 0.0,
                                  const double& lateral_offset = 0.0) {
    // Update Cylindrical position
    this->cylindrical_position = cylindrical_position;

    // Update Cartesian position
    const double control_view_angle = cylindrical_position.yaw - control_view_offset;
    cartesian_position.x =
        cylindrical_position.radius * std::cos(control_view_angle)
        - lateral_offset * std::sin(control_view_angle);
    cartesian_position.y =
        cylindrical_position.radius * std::sin(control_view_angle)
        + lateral_offset * std::cos(control_view_angle);
    cartesian_position.z = cylindrical_position.height;

    // Update orientation
    RPY current_rpy;  current_rpy.convertFromQuaternion(orientation);
    RPY pose_rpy;
    pose_rpy.roll = current_rpy.roll;
    pose_rpy.pitch = current_rpy.pitch;
    pose_rpy.yaw = cylindrical_position.yaw;
    orientation = pose_rpy.convertToQuaternion();
  }

  geometry_msgs::Pose toCartesianPose() const {
    geometry_msgs::Pose pose;
    pose.position = cartesian_position;
    pose.orientation = orientation;
    return pose;
  }

  geometry_msgs::PoseStamped toCartesianPoseStamped() const {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = this->toCartesianPose();
    return pose_stamped;
  }
};

#endif // #ifndef MARSLITE_CONTROL_UTILS_CYLINDRICAL_POSE_H