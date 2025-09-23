#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "utils/tf2_listener_wrapper.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_pose_publisher_node");
  ros::NodeHandle nh;
  ros::Publisher gripper_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(
      "/marslite_control/gripper_pose", 1
  );
  ros::Rate loop_rate(10);

  Tf2ListenerWrapper tf2_listener;
  geometry_msgs::TransformStamped gripper_transform;
  geometry_msgs::PoseStamped gripper_pose;
  while (ros::ok()) {
    gripper_transform = tf2_listener.lookupTransform("base_link", "tm_gripper");
    gripper_pose.header.frame_id = "base_link";
    gripper_pose.header.stamp = ros::Time::now();
    gripper_pose.pose.position.x = gripper_transform.transform.translation.x;
    gripper_pose.pose.position.y = gripper_transform.transform.translation.y;
    gripper_pose.pose.position.z = gripper_transform.transform.translation.z;
    gripper_pose.pose.orientation = gripper_transform.transform.rotation;
    gripper_pose_publisher.publish(gripper_pose);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}