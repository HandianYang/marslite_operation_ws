#include "cartesian_control/joystick_teleoperation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_joystick_teleoperation");
  JoystickTeleoperationWrapper joystick_teleoperation;
  joystick_teleoperation.teleoperate();
  
  ros::shutdown();
  return 0;
}