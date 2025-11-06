#include "cartesian_control/motion_controller_teleoperation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_controller_teleoperation_node");
  MotionControllerTeleoperation motion_controller_teleoperation;
  ros::Duration(1.0).sleep(); // wait for a while to let ROS setup completely
  
  // Gripper initial pose options are:
  // - motion_controller_teleoperation.resetToFrontPose();
  // - motion_controller_teleoperation.resetToLeftPose();
  // - motion_controller_teleoperation.resetToRightPose();
  // Change to your desired initial pose here: 
  motion_controller_teleoperation.resetToLeftPose();
  motion_controller_teleoperation.run();
  return 0;
}