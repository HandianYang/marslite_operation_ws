#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "utils/tf2_listener_wrapper.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_pose_publisher_node");
  ros::NodeHandle nh;
  ros::Publisher gripper_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(
      "/marslite_control/current_gripper_pose", 1
  );
  ros::Rate loop_rate(10);

  Tf2ListenerWrapper tf2_listener;
  geometry_msgs::TransformStamped current_gripper_transform;
  geometry_msgs::PoseStamped current_gripper_pose;
  while (ros::ok()) {
    current_gripper_transform = tf2_listener.lookupTransform("base_link", "tm_gripper");
    current_gripper_pose.header = current_gripper_transform.header;
    current_gripper_pose.pose.position.x = current_gripper_transform.transform.translation.x;
    current_gripper_pose.pose.position.y = current_gripper_transform.transform.translation.y;
    current_gripper_pose.pose.position.z = current_gripper_transform.transform.translation.z;
    current_gripper_pose.pose.orientation = current_gripper_transform.transform.rotation;
    gripper_pose_publisher.publish(current_gripper_pose);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}