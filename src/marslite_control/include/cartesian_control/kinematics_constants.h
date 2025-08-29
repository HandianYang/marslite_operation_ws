#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H

#include <geometry_msgs/PoseStamped.h>

// initial gripper pose toward front (corresponds to button-Y pose)
// [direction] GripperDirection::FRONT
// [base_frame] /base_link
// [end_effector_frame] /tm_gripper
const geometry_msgs::PoseStamped kFrontInitialGripperPose = [] {
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

// initial gripper pose toward left (corresponds to button-X pose)
// [direction] GripperDirection::LEFT
// [base_frame] /base_link
// [end_effector_frame] /tm_gripper
const geometry_msgs::PoseStamped kLeftInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.472;
  pose_stamped.pose.position.y = 0.627;
  pose_stamped.pose.position.z = 0.887;
  pose_stamped.pose.orientation.x = 0.000;
  pose_stamped.pose.orientation.y = 0.844;
  pose_stamped.pose.orientation.z = 0.536;
  pose_stamped.pose.orientation.w = 0.000;
  return pose_stamped;
}();

// initial gripper pose toward right
// [direction] GripperDirection::RIGHT
// [base_frame] /base_link
// [end_effector_frame] /tm_gripper
const geometry_msgs::PoseStamped kRightInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.293;
  pose_stamped.pose.position.y = -0.656;
  pose_stamped.pose.position.z = 0.948;
  pose_stamped.pose.orientation.x = 0.711;
  pose_stamped.pose.orientation.y = 0.002;
  pose_stamped.pose.orientation.z = 0.000;
  pose_stamped.pose.orientation.w = 0.703;
  return pose_stamped;
}();

#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H