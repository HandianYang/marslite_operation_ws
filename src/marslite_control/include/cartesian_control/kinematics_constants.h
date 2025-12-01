#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H

#include <geometry_msgs/PoseStamped.h>

namespace marslite {

// TODO: Remove this feature
enum class OperatingDirection : uint8_t {
  FRONT = 0,
  LEFT = 1,
  RIGHT = 2
};

/**
 * @brief Properties for real robot operation
 * @note The gripper pose is defined under `/tm_base` -> `/tm_gripper`
 */
namespace real {

const geometry_msgs::PoseStamped kFrontInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "tm_base";
  pose_stamped.pose.position.x = 0.538;
  pose_stamped.pose.position.y = -0.122;
  pose_stamped.pose.position.z = 0.666;
  pose_stamped.pose.orientation.x = 0.510;
  pose_stamped.pose.orientation.y = 0.510;
  pose_stamped.pose.orientation.z = 0.490;
  pose_stamped.pose.orientation.w = 0.490;
  return pose_stamped;
}();

const geometry_msgs::PoseStamped kLeftInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "tm_base";
  pose_stamped.pose.position.x = 0.122;
  pose_stamped.pose.position.y = 0.538;
  pose_stamped.pose.position.z = 0.666;
  pose_stamped.pose.orientation.x = -0.000;
  pose_stamped.pose.orientation.y = 0.721;
  pose_stamped.pose.orientation.z = 0.693;
  pose_stamped.pose.orientation.w = -0.000;
  return pose_stamped;
}();

const geometry_msgs::PoseStamped kLeftFrontInitialGripperPose = [] {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "tm_base";
  pose_stamped.pose.position.x = 0.466692;
  pose_stamped.pose.position.y = 0.294154;
  pose_stamped.pose.position.z = 0.666;
  pose_stamped.pose.orientation.x = 0.27585;
  pose_stamped.pose.orientation.y = 0.665961;
  pose_stamped.pose.orientation.z = 0.640351;
  pose_stamped.pose.orientation.w = 0.265242;
  return pose_stamped;
}();

// TODO: Modify value and base frame needed
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

// TODO: Modify base frame for simulation

/**
 * @brief Properties for simulation operation
 * @note The gripper pose is defined under `/base_link` -> `/robotiq_85_base_link`
 */
namespace sim {

// TODO: Modify value needed
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

// TODO: Modify value needed
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

} // namespace sim

} // namespace marslite



#endif // #ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KINEMATICS_CONSTANTS_H