/**
 * @file test_detected_object_reaching.cpp
 * @brief
 * To test whether the end-effector frame can reach center point of
 * every detected object's surface.
 * 
 * This test script receives detected objects from a YOLO inference node,
 * transforms their centroid positions to the robot base frame (/tm_base),
 * and publishes the target frame (/target_frame) to guide the robot's
 * end-effector to reach each detected object sequentially and repeatedly.
 * 
 * @note
 * This node requires the following nodes to be launched:
 * 1. YOLO inference node with camera bringup (choose one):
 *  - detection_examples/yolo_inference_bbox_estimator
 *  - detection_examples/yolo_inference_centroid_estimator
 * 2. Robot bringup with cartesian controllers:
 *  - controller_bringup/motion_controller_bringup.launch
 * 3. TF broadcaster for /camera_link to /tm_tip_link:
 *  - easy_handeye/eye_in_hand_publish.launch
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "detection_msgs/DetectedObjectArray.h"
#include "utils/tf2_listener_wrapper.h"

class TestNode {
 public:
  explicit TestNode(const ros::NodeHandle& nh = ros::NodeHandle())
      : nh_(nh) {
    detected_objects_subscriber_ = nh_.subscribe(
        "/yolo/detected_objects", 1,
        &TestNode::detectedObjectsCallback, this);
    target_frame_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/target_frame", 1);
    
    initial_pose_ = tf2_listener_.lookupTransform<geometry_msgs::PoseStamped>(
        "tm_base", "tm_gripper");
    target_pose_ = initial_pose_;
  }

  void run() {
    while (ros::ok()) {
      for (const auto& obj : detected_objects_in_tm_base_.objects) {
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "tm_base";
        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position = obj.centroid;
        target_pose.pose.orientation = initial_pose_.pose.orientation;

        target_frame_publisher_.publish(target_pose);
        ROS_INFO_STREAM("Published target frame at position: "
                        << target_pose.pose.position.x << ", "
                        << target_pose.pose.position.y << ", "
                        << target_pose.pose.position.z);
        ros::Duration(10.0).sleep();

        ROS_INFO("Returning to initial pose.");
        target_frame_publisher_.publish(initial_pose_);
        ros::Duration(10.0).sleep();
      }

      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }

 private:
  void detectedObjectsCallback(const detection_msgs::DetectedObjectArray::ConstPtr& objects) {
    detected_objects_in_tm_base_ = *objects;
    geometry_msgs::TransformStamped camera_to_tm_base_tf = 
        tf2_listener_.lookupTransform<geometry_msgs::TransformStamped>("tm_base", "camera_color_optical_frame");
    for (auto& obj : detected_objects_in_tm_base_.objects) {
      geometry_msgs::PoseStamped pose_in_camera_link;
      pose_in_camera_link.header.frame_id = obj.frame;
      pose_in_camera_link.pose.position = obj.centroid;
      pose_in_camera_link.pose.orientation.w = 1.0;

      geometry_msgs::PoseStamped pose_in_tm_base;
      tf2::doTransform(pose_in_camera_link, pose_in_tm_base, camera_to_tm_base_tf);

      obj.frame = "tm_base";
      obj.centroid = pose_in_tm_base.pose.position;
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher target_frame_publisher_;
  ros::Subscriber detected_objects_subscriber_;
  geometry_msgs::PoseStamped initial_pose_;
  geometry_msgs::PoseStamped target_pose_;
  Tf2ListenerWrapper tf2_listener_;
  detection_msgs::DetectedObjectArray detected_objects_in_tm_base_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_detected_object_reaching");
  TestNode test_node;
  test_node.run();
  return 0;
}