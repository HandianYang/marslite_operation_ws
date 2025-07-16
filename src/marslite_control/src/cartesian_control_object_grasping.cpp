#include "cartesian_control/cartesian_control_object_grasping.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

CartesianControlObjectGrasping::CartesianControlObjectGrasping(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(25))
{
  target_frame_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 10);
  gripper_pub_ = nh_.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
  detected_objects_sub_ = nh_.subscribe("/yolo/detected_objects", 10, &CartesianControlObjectGrasping::detectedObjectsCallback, this);

  drive_to_initial_point_service_ = nh_.advertiseService(
      "/drive_to_initial_point", &CartesianControlObjectGrasping::driveToInitialPoint, this);
  drive_to_observe_point_service_ = nh_.advertiseService(
      "/drive_to_observe_point", &CartesianControlObjectGrasping::driveToObservePoint, this);
  record_object_position_service_ = nh_.advertiseService(
      "/record_object_position", &CartesianControlObjectGrasping::recordObjectPosition, this);
  grasp_object_with_id_service_ = nh_.advertiseService(
      "/grasp_object_with_id", &CartesianControlObjectGrasping::graspObjectWithId, this);
}