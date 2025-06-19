#include "cartesian_control/cartesian_control_object_grasping.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_cartesian_control_object_grasping");
  CartesianControlObjectGrasping object_grasping;
  ros::spin();
  return 0;
}