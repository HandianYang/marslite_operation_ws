#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "cartesian_control/kinematics_constants.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_cartesian_control");
  ros::NodeHandle nh;
  ros::Publisher target_frame_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "/target_frame", 1);
  
  // Change your initial_pose and target_pose here for testing
  //  (e.g.) `marslite::sim::kFrontInitialGripperPose` for simulation
  geometry_msgs::PoseStamped initial_pose = marslite::real::kFrontInitialGripperPose;
  target_frame_pub.publish(initial_pose);
  ros::Duration(3.0).sleep();  // Allow time for the message to be sent

  geometry_msgs::PoseStamped target_pose = marslite::real::kLeftInitialGripperPose;
  target_frame_pub.publish(target_pose);

  ROS_INFO("test_cartesian_control completed.");
  return 0;
}