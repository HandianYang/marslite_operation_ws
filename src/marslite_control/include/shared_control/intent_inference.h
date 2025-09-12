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

// proximity likelihood parameter
const double kKappa = 3.0;  
// confidence threshold to lock target
const double kLockTargetConfidenceThreshold = 0.3;

enum GripperMotionState {
  IDLE = 0,
  APPROACHING = 1,
  RETREATING = 2,
  GRASPING = 3,
  NEAR_GOAL = 4
};

class IntentInference {
 public:
  IntentInference(const double& transition_probability = 0.1,
                  const double& confidence_lower_bound = 0.3,
                  const double& confidence_upper_bound = 0.9,
                  const double& alpha_maximum = 0.7);

  void setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects);

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
    // TODO: handle empty belief_
    const auto max_it = std::max_element(belief_.begin(), belief_.end(),
                                        [](const auto& a, const auto& b) {
                                            return a.second < b.second;
                                        });
    return max_it->first;
  }

  inline geometry_msgs::Point getMostLikelyGoalPosition() const {
    return position_to_base_link_.at(this->getMostLikelyGoal());
  }

  inline double getConfidence() const {
    return confidence_;
  }

  inline double getAlpha() const {
    return alpha_;
  }

  inline detection_msgs::ObjectBeliefMap getBelief() const {
    return belief_;
  }

  inline const bool isApproaching() const {
    return gripper_motion_state_ == GripperMotionState::APPROACHING;
  }

  visualization_msgs::MarkerArray getBeliefVisualization();

  geometry_msgs::Point getGoalDirection(const detection_msgs::DetectedObject& goal) const;

  void updateBelief();

 private:
  void updatePositionToBaseLink();
  geometry_msgs::Point transformOdomToBaseLink(const geometry_msgs::Point& point_in_odom);

  void updateGripperMotionState();
  bool isNearAnyGoal() const;
  bool isTowardAnyGoal() const;
  
  double getCosineSimilarity(const geometry_msgs::Point& user_direction,
                             const geometry_msgs::Point& goal_direction) const;
  void normalizeBelief();
  void calculateConfidence();
  void calculateAlpha();

  // constants
  double transition_probability_; // [0,1], constant
  double confidence_lower_bound_; // [0,1], constant
  double confidence_upper_bound_; // [0,1], constant
  double alpha_maximum_; // [0,1], constant

  double confidence_; // [0,1]
  double alpha_;  // [0,1]
  geometry_msgs::Point gripper_position_;
  geometry_msgs::Point gripper_direction_;
  std_msgs::Bool gripper_status_;
  detection_msgs::DetectedObjectArray recorded_objects_;
  detection_msgs::ObjectBeliefMap belief_;
  detection_msgs::ObjectPositionMap position_to_base_link_;
  Tf2ListenerWrapper tf2_listener_;
  GripperMotionState gripper_motion_state_;
};

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
