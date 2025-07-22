#ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROLLER_H
#define MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

class SharedController {
public:
  explicit SharedController(const ros::NodeHandle& nh = ros::NodeHandle());

private:
  ros::NodeHandle nh_;
  ros::Publisher desired_gripper_pose_publisher_;
  ros::Publisher desired_gripper_status_publisher_;
  ros::Subscriber user_desired_gripper_pose_subscriber_;
  ros::Subscriber user_desired_gripper_status_subscriber_;

  geometry_msgs::PoseStamped user_desired_gripper_pose_;
  std_msgs::Bool user_desired_gripper_status_;

  bool use_shared_controller_;  // false if using pure teleoperation

private:
  void initializePublishers();
  void initializeSubscribers();
  void parseParameters();

  inline void userDesiredGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    user_desired_gripper_pose_ = *msg;
    if (!use_shared_controller_)
      desired_gripper_pose_publisher_.publish(user_desired_gripper_pose_);
  }

  inline void userDesiredGripperStatusCallback(const std_msgs::Bool::ConstPtr& msg) {
    // Directly publish the user's desired gripper status becuase we don't 
    //   need to process it through shared control.
    user_desired_gripper_status_ = *msg;
    desired_gripper_status_publisher_.publish(user_desired_gripper_status_);
  }
};

#endif // #ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROLLER_H