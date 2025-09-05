#ifndef MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
#define MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <map>

#include "detection_msgs/DetectedObject.h"
#include "detection_msgs/DetectedObjectArray.h"
#include "utils/tf2_listener_wrapper.h"


namespace detection_msgs{

struct DetectedObjectComparator {
  inline const bool operator()(
      const detection_msgs::DetectedObject& a,
      const detection_msgs::DetectedObject& b) const {
    return a.centroid.x < b.centroid.x;
  }
};

using ObjectBeliefMap = std::map<detection_msgs::DetectedObject, double, DetectedObjectComparator>;
using ObjectPositionMap = std::map<detection_msgs::DetectedObject, geometry_msgs::Point, DetectedObjectComparator>;

} // namespace detection_msgs


const double kKappa = 3.0;  // proximity likelihood parameter

enum GripperMotionState {
  IDLE = 0,
  APPROACHING = 1,
  RETREATING = 2,
  GRASPING = 3,
  NEAR_GOAL = 4
};

class IntentInference {
public:
  IntentInference(const double& transition_probability = 0.1);
  IntentInference(const detection_msgs::DetectedObjectArray& recorded_objects,
                  const double& transition_probability = 0.1);

  inline void setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects) {
    recorded_objects_ = recorded_objects;
    belief_ = {};
    position_to_base_link_ = {};
    for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
      belief_[obj] = 1.0 / recorded_objects.objects.size();
      position_to_base_link_[obj] = this->transformOdomToBaseLink(obj.centroid);
    }
  }

  inline void setGripperPosition(const geometry_msgs::Point& position) {
    gripper_position_ = position;
  }
  
  inline void setGripperDirection(const geometry_msgs::Point& direction) {
    gripper_direction_ = direction;
  }

  inline void setGripperStatus(const std_msgs::Bool& status) {
    gripper_status_ = status;
  }

  inline detection_msgs::DetectedObject getMostLikelyGoal() const {
    const auto max_it = std::max_element(belief_.begin(), belief_.end(),
                                        [](const auto& a, const auto& b) {
                                            return a.second < b.second;
                                        });
    return max_it->first;
  }

  inline detection_msgs::ObjectBeliefMap getBelief() const {
    return belief_;
  }

  visualization_msgs::MarkerArray getBeliefVisualization();

  void updatePositionToBaseLink();
  void updateGripperMotionState();
  void updateBelief();

private:
  double transition_probability_;
  geometry_msgs::Point gripper_position_;
  geometry_msgs::Point gripper_direction_;
  std_msgs::Bool gripper_status_;
  detection_msgs::DetectedObjectArray recorded_objects_;
  detection_msgs::ObjectBeliefMap belief_;
  detection_msgs::ObjectPositionMap position_to_base_link_;
  Tf2ListenerWrapper tf2_listener_;
  GripperMotionState gripper_motion_state_;
  
  geometry_msgs::Point transformOdomToBaseLink(const geometry_msgs::Point& point_in_odom);
  bool isTowardAnyGoal() const;
  bool isNearAnyGoal() const;
  geometry_msgs::Point getGoalDirection(const detection_msgs::DetectedObject& goal) const;
  double getCosineSimilarity(const geometry_msgs::Point& user_direction, const geometry_msgs::Point& goal_direction) const;
  void normalizeBelief();
};

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
