#include "cartesian_control/cartesian_control_keyboard.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cartesian_control_keyboard");
  CartesianControlKeyboard cartesian_control_keyboard;
  cartesian_control_keyboard.run();
  ros::shutdown();
  return 0;
}