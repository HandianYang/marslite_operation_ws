#include "cartesian_control/cartesian_control_joystick.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cartesian_control_joystick");
  CartesianControlJoystick cartesian_control_joystick;
  cartesian_control_joystick.run();
  ros::shutdown();
  return 0;
}