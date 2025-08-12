#include "cartesian_control/keyboard_teleoperation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_teleoperation_node");
  KeyboardTeleoperation keyboard_teleoperation;
  keyboard_teleoperation.teleoperate();

  ros::shutdown();
  return 0;
}