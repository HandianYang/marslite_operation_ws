#include "shared_control/shared_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shared_control_node");  
  SharedControl shared_control;
  shared_control.run_inference();
  return 0;
}