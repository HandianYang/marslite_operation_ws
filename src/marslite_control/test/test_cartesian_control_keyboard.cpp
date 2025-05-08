// #include <ros/ros.h>
// #include <iomanip>
// #include <termios.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <sys/time.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/Joy.h>
#include "cartesian_control/cartesian_control_keyboard.h"

// const static double kTriggerThreshold = 0.95;
// bool is_hand_trigger_pressed = false;
// bool is_index_trigger_pressed = false;
// bool is_button_X_pressed_over_3s_ = false;
// bool is_button_Y_pressed_ = false;

// geometry_msgs::PoseStamped previous_left_joy_pose;
// geometry_msgs::PoseStamped current_left_joy_pose;

// geometry_msgs::PoseStamped initial_gripper_pose;
// geometry_msgs::PoseStamped relative_gripper_pose;
// geometry_msgs::PoseStamped target_gripper_pose;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cartesian_control_keyboard");
  CartesianControlKeyboard cartesian_control_keyboard;
  cartesian_control_keyboard.run();
  // return 0;


  // ros::NodeHandle nh;
  // ros::Publisher target_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("target_frame", 10);
  // ros::Subscriber left_joy_pose_sub = nh.subscribe("/unity/joy_pose/left", 10, leftJoyPoseCallback);
  // ros::Subscriber left_joy_sub = nh.subscribe("/unity/joy/left", 10, leftJoyCallback);
  // ros::Rate loop_rate(25);

  // initial_gripper_pose.header.frame_id = "base_link";
  // initial_gripper_pose.pose.position.x = 0.712;
  // initial_gripper_pose.pose.position.y = -0.122;
  // initial_gripper_pose.pose.position.z = 1.043;
  // initial_gripper_pose.pose.orientation.x = 0.5;
  // initial_gripper_pose.pose.orientation.y = 0.5;
  // initial_gripper_pose.pose.orientation.z = 0.5;
  // initial_gripper_pose.pose.orientation.w = 0.5;
  // target_gripper_pose = initial_gripper_pose;
  // target_frame_pub.publish(initial_gripper_pose);

  // bool begin_teleoperation = true;
  // while (ros::ok()) {
  //   if (is_index_trigger_pressed && is_hand_trigger_pressed) {
  //     if (begin_teleoperation) {
  //       ROS_INFO_STREAM("Begin the teleoperation...");
  //       previous_left_joy_pose.pose = current_left_joy_pose.pose;
  //       begin_teleoperation = false;
  //     }

  //     relative_gripper_pose.pose.position.x = current_left_joy_pose.pose.position.x - previous_left_joy_pose.pose.position.x;
  //     relative_gripper_pose.pose.position.y = current_left_joy_pose.pose.position.y - previous_left_joy_pose.pose.position.y;
  //     relative_gripper_pose.pose.position.z = current_left_joy_pose.pose.position.z - previous_left_joy_pose.pose.position.z;

  //     target_gripper_pose.pose.position.x = initial_gripper_pose.pose.position.x + relative_gripper_pose.pose.position.x * 0.8;
  //     target_gripper_pose.pose.position.y = initial_gripper_pose.pose.position.y + relative_gripper_pose.pose.position.y * 0.8;
  //     target_gripper_pose.pose.position.z = initial_gripper_pose.pose.position.z + relative_gripper_pose.pose.position.z * 0.8;
  //     target_frame_pub.publish(target_gripper_pose);
  //   } else {
  //     begin_teleoperation = true;
  //     initial_gripper_pose = target_gripper_pose;
  //   }

  //   loop_rate.sleep();
  //   ros::spinOnce();
  // }

  return 0;
}