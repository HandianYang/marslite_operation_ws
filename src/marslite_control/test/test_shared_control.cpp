#include "shared_control/shared_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_shared_controller");  
  SharedController shared_controller;

  ros::spin();
  return 0;
}