#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class CartesianControlKeyboard {
public:
  explicit CartesianControlKeyboard(const ros::NodeHandle& nh = ros::NodeHandle());
  void run();

private:
  void getKeyboardInput();
  void processInput();
  void publishPose();

  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher pose_pub_;
  ros::Publisher target_frame_pub_;

  geometry_msgs::PoseStamped current_pose_;
  char input_key_;
  bool is_stopped_;
};

#endif // MARSLITE_CONTROL_CARTESIAN_CONTROL_KEYBOARD_H