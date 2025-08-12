#include "cartesian_control/motion_controller_teleoperation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_controller_teleoperation_node");
  MotionControllerTeleoperation motion_controller_teleoperation;
  motion_controller_teleoperation.teleoperate();
  
  ros::shutdown();
  return 0;
}