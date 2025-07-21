#include "cartesian_control/object_grasping.h"

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

ObjectGraspingWrapper::ObjectGraspingWrapper(const ros::NodeHandle& nh)
    : nh_(nh), rate_(ros::Rate(25))
{
  target_frame_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 10);
  detected_objects_subscriber_ = nh_.subscribe("/yolo/detected_objects", 10,
      &ObjectGraspingWrapper::detectedObjectsCallback, this);
}