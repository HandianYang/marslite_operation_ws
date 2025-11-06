#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H

#include <geometry_msgs/PoseStamped.h>

namespace marslite {

enum class OperatingDirection : uint8_t {
  FRONT = 0,
  LEFT = 1,
  RIGHT = 2
};

/**
 * @brief Properties for real robot operation
 * @note The gripper pose is defined under `/base_link` -> `/tm_gripper`
 */
namespace real {

// [direction] OperatingDirection::FRONT
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

// [direction] OperatingDirection::LEFT
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

// [direction] OperatingDirection::RIGHT
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
 
} // namespace real


/**
 * @brief Properties for simulation operation
 * @note The gripper pose is defined under `/base_link` -> `/robotiq_85_base_link`
 */
namespace sim {

// [direction] OperatingDirection::FRONT
const geometry_msgs::PoseStamped kFrontInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.538;
  pose_stamped.pose.position.y = -0.122;
  pose_stamped.pose.position.z = 0.968;
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;
  return pose_stamped;
}();

// [direction] OperatingDirection::LEFT
const geometry_msgs::PoseStamped kLeftInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose.position.x = 0.472;
  pose_stamped.pose.position.y = 0.109;
  pose_stamped.pose.position.z = 0.898;
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.707;
  pose_stamped.pose.orientation.w = 0.707;
  return pose_stamped;
}();

// TODO: define kRightInitialGripperPose for simulation

} // namespace sim

} // namespace marslite



#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H