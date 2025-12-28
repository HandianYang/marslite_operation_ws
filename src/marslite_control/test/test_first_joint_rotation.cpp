/**
 * @file test_first_joint_rotation.cpp
 * @brief
 * To test the functionality of first joint rotation in
 * MotionControllerTeleoperation::generateInitialGripperPose().
 *    
 * This test script initializes robot's gripper pose from the front pose, and
 * gradually rotates the first joint from 0 to 90 degrees, publishing the
 * target pose to the /target_frame topic.
 *        
 * @note
 * This node requires the following nodes to be launched:
 * 1. Robot bringup with cartesian controllers:
 *  - controller_bringup/motion_controller_bringup.launch
 */

#include "cartesian_control/motion_controller_teleoperation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_first_joint_rotation");
  ros::NodeHandle nh;
  ros::Publisher target_frame_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "/target_frame", 1);
  
  //// Change parameters here: ////
  const bool use_sim = false;
  const double control_view_offset = use_sim ? 0.0 : M_PI / 2;
  /////////////////////////////////


  /////////// Test code ///////////
  ROS_INFO("Start testing first joint rotation...");
  for (int degree = 0; degree <= 90; degree += 1) {
    geometry_msgs::PoseStamped target_pose =
        MotionControllerTeleoperation::generateInitialGripperPose(
            degree * M_PI / 180.0, control_view_offset, use_sim
    );
    target_frame_pub.publish(target_pose);

    ros::spinOnce();
    if (degree == 0)
      ros::Duration(10.0).sleep();
    else
      ros::Duration(0.1).sleep();
  }
  ROS_INFO("Test completed.");
  /////////////////////////////////

  return 0;
}