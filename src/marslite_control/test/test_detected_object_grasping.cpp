#include "cartesian_control/object_grasping.h"

#include "detection_msgs/DetectedObject.h"
#include "detection_msgs/DetectedObjectArray.h"
#include "detection_msgs/DriveToInitialPoint.h"
#include "detection_msgs/DriveToObservePoint.h"
#include "detection_msgs/GraspObjectWithID.h"
#include "detection_msgs/RecordObjectPosition.h"
#include "cartesian_control/kinematics_constants.h"

class TestingServiceWrapper {
public:
  explicit TestingServiceWrapper(const ros::NodeHandle& nh = ros::NodeHandle())
      : nh_(nh), object_grasping_wrapper_(nh) {
    drive_to_initial_point_service_ = nh_.advertiseService(
        "/drive_to_initial_point", &TestingServiceWrapper::driveToInitialPoint, this);
    drive_to_observe_point_service_ = nh_.advertiseService(
        "/drive_to_observe_point", &TestingServiceWrapper::driveToObservePoint, this);
    record_object_position_service_ = nh_.advertiseService(
        "/record_object_position", &TestingServiceWrapper::recordObjectPosition, this);
    grasp_object_with_id_service_ = nh_.advertiseService(
        "/grasp_object_with_id", &TestingServiceWrapper::graspObjectWithId, this);
  }

private:
  ObjectGraspingWrapper object_grasping_wrapper_;
  detection_msgs::DetectedObjectArray recorded_objects_;

  ros::NodeHandle nh_;
  ros::ServiceServer drive_to_initial_point_service_;
  ros::ServiceServer drive_to_observe_point_service_;
  ros::ServiceServer record_object_position_service_;
  ros::ServiceServer grasp_object_with_id_service_;

private:
  inline bool driveToInitialPoint(
      detection_msgs::DriveToInitialPoint::Request& req,
      detection_msgs::DriveToInitialPoint::Response& res) {
    object_grasping_wrapper_.publishTargetFrame(kInitialGripperPose);
    return true;
  }

  inline bool driveToObservePoint(
      detection_msgs::DriveToObservePoint::Request& req,
      detection_msgs::DriveToObservePoint::Response& res) {
    object_grasping_wrapper_.publishTargetFrame(kObservingGripperPose);
    return true;
  }

  inline bool recordObjectPosition(
      detection_msgs::RecordObjectPosition::Request& req,
      detection_msgs::RecordObjectPosition::Response& res) {
    recorded_objects_ = object_grasping_wrapper_.getDetectedObjects();
    if (recorded_objects_.objects.empty()) {
      res.result = "[Warning] No objects detected to record.";
      res.success = false;
      return false;
    }

    std::ostringstream oss;
    oss << recorded_objects_.objects.size() << " objects detected: ";
    for (const auto& object : recorded_objects_.objects) {
      oss << "[" << std::fixed << std::setprecision(3)
          << object.centroid.x << ", "
          << object.centroid.y << ", "
          << object.centroid.z << "]  ";
    }
    res.result = oss.str();
    res.success = true;
    return true;
  }

  inline bool graspObjectWithId(
      detection_msgs::GraspObjectWithID::Request& req,
      detection_msgs::GraspObjectWithID::Response& res) {
    if (req.id < 0 || req.id >= static_cast<int>(recorded_objects_.objects.size())) {
      res.result = "[Error] Invalid object ID! Skip grasping.";
      res.success = false;
      return false;
    }

    geometry_msgs::PoseStamped target_gripper_pose;
    target_gripper_pose.header.frame_id = "base_link";
    target_gripper_pose.header.stamp = ros::Time::now();
    target_gripper_pose.pose.position = recorded_objects_.objects[req.id].centroid;
    target_gripper_pose.pose.orientation = kInitialGripperPose.pose.orientation;
    object_grasping_wrapper_.publishTargetFrame(target_gripper_pose);

    res.result = "Grasping object with ID " + std::to_string(req.id);
    res.success = true;
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_detected_object_grasping");
  TestingServiceWrapper testing_service_wrapper;
  ros::spin();
  return 0;
}