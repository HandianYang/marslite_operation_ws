#include "cartesian_control/motion_controller_teleoperation.h"
#include "cartesian_control/kinematics_constants.h"

static void resetToFrontPose(MotionControllerTeleoperation& motion_controller_teleoperation) {
  motion_controller_teleoperation.initializeGripperPose(kFrontInitialGripperPose);
  motion_controller_teleoperation.setGripperDirection(GripperDirection::FRONT);
  ROS_INFO("Wait for 10 seconds to start teleoperation...");
  ros::Duration(10.0).sleep();
}

static void resetToLeftPose(MotionControllerTeleoperation& motion_controller_teleoperation) {
  motion_controller_teleoperation.initializeGripperPose(kLeftInitialGripperPose);
  motion_controller_teleoperation.setGripperDirection(GripperDirection::LEFT);
  ROS_INFO("Wait for 10 seconds to start teleoperation...");
  ros::Duration(10.0).sleep();
}

static void resetToRightPose(MotionControllerTeleoperation& motion_controller_teleoperation) {
  motion_controller_teleoperation.initializeGripperPose(kRightInitialGripperPose);
  motion_controller_teleoperation.setGripperDirection(GripperDirection::RIGHT);
  ROS_INFO("Wait for 10 seconds to start teleoperation...");
  ros::Duration(10.0).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_controller_teleoperation_node");
  MotionControllerTeleoperation motion_controller_teleoperation;
  ros::Duration(2.0).sleep();
  
  // resetToFrontPose(motion_controller_teleoperation);
  resetToLeftPose(motion_controller_teleoperation);
  // resetToRightPose(motion_controller_teleoperation);

  motion_controller_teleoperation.teleoperate();
  
  ros::shutdown();
  return 0;
}