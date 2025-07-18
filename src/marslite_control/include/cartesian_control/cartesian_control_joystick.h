#ifndef MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H
#define MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TfListenerWrapper {
public:
  TfListenerWrapper(): tf_buffer_(), tf_listener_(tf_buffer_) {}
  
  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame)
  {
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
              
      // ROS_INFO_STREAM("Transform from " << source_frame << " to " << target_frame << ":");
      // ROS_INFO_STREAM(transformStamped.transform.translation.x << ", "
      //                 << transformStamped.transform.translation.y << ", "
      //                 << transformStamped.transform.translation.z);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s. Return empty TransformStamped() instead.", ex.what());
    }
    return transform_stamped;
}
  
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

// initial gripper pose (corresponds to button-Y pose)
//   base_frame: /base_link
//   end_effector_frame: /tm_gripper
const geometry_msgs::PoseStamped kInitialGripperPose = [] {
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

const static double kTriggerThreshold = 0.95;

struct RPY {
  double roll;
  double pitch;
  double yaw;
};

class CartesianControlJoystick {
public:
  explicit CartesianControlJoystick(const ros::NodeHandle& nh = ros::NodeHandle());

  inline void teleoperate()
  {
    while (nh_.ok()) {
      this->teleoperateOnce();
      rate_.sleep();
      ros::spinOnce();
    }
  }
  void teleoperateOnce();
  void calculateTargetGripperPose();
  
  inline void setGripperPositionToCurrent()
  {
    target_gripper_pose_.pose.position.x = base_link_to_tm_gripper_transform_.transform.translation.x;
    target_gripper_pose_.pose.position.y = base_link_to_tm_gripper_transform_.transform.translation.y;
    target_gripper_pose_.pose.position.z = base_link_to_tm_gripper_transform_.transform.translation.z;
  }

  inline void setGripperOrientationToCurrent()
  {
    target_gripper_pose_.pose.orientation = base_link_to_tm_gripper_transform_.transform.rotation;
  }

  inline void publishTargetGripperPose(
      const geometry_msgs::PoseStamped& target_gripper_pose = kInitialGripperPose)
  {
    target_frame_publisher_.publish(target_gripper_pose);
  }

  inline geometry_msgs::PoseStamped getTargetGripperPose() const
  {
    return target_gripper_pose_;
  }

private:
  // ROS mechanisms
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher target_frame_publisher_;
  ros::Publisher gripper_publisher_;
  ros::Subscriber left_joy_pose_subscriber_;
  ros::Subscriber left_joy_subscriber_;
  TfListenerWrapper tf2_listener_;

  // ROS messages
  geometry_msgs::PoseStamped initial_left_joy_pose_;
  geometry_msgs::PoseStamped current_left_joy_pose_;
  geometry_msgs::PoseStamped initial_gripper_pose_;
  geometry_msgs::PoseStamped target_gripper_pose_;
  geometry_msgs::TransformStamped base_link_to_tm_gripper_transform_;
  std_msgs::Bool gripper_;

  // flags
  bool is_begin_teleoperation_;
  bool is_position_change_enabled_;
  bool is_orientation_change_enabled_;

  // parameters
  double position_scale_;
  double orientation_scale_;

private:
  // initialization
  void parseParameters();
  void initializePublishersAndSubscribers();
  void setInitialGripperPose();

  // main operations (supports calculateTargetGripperPose())
  geometry_msgs::Vector3 getPositionDifference();
  RPY getOrientationDifference();
  RPY getRPYFromPose(const geometry_msgs::PoseStamped& pose);
  double restrictAngleWithinPI(const double& angle);

  // callbacks
  void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif // MARSLITE_CONTROL_CARTESIAN_CONTROL_JOYSTICK_H