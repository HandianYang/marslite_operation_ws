#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H

#include <geometry_msgs/PoseStamped.h>

// initial gripper pose (corresponds to button-Y pose)
// [base_frame] /base_link
// [end_effector_frame] /tm_gripper
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
// [base_frame] /base_link
// [end_effector_frame] /tm_gripper
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

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H