#ifndef MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
#define MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
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

enum class GripperMotionState : uint8_t {
  IDLE = 0,
  APPROACH = 1,
  RETREAT = 2,
  LOCKED = 3,
  REACHED = 4,
  GRASPED = 5
};

static std::string toString(GripperMotionState s) {
  switch(s) {
    case GripperMotionState::IDLE:  return "IDLE";
    case GripperMotionState::APPROACH:  return "APPROACH";
    case GripperMotionState::RETREAT: return "RETREAT";
    case GripperMotionState::LOCKED:  return "LOCKED";
    case GripperMotionState::REACHED: return "REACHED";
    case GripperMotionState::GRASPED: return "GRASPED";
  }
  return "UNKNOWN"; // fallback
}


class IntentInference {
 public:
  // proximity likelihood parameter
  static inline constexpr double kProximityLikelihoodParameter = 3.0;
  // confidence threshold to lock target
  static inline constexpr double kLockTargetConfidenceThreshold = 0.3;
  // [m/s] speed threshold to consider "GripperMotionState::IDLE"
  static inline constexpr double kIdleSpeedThreshold = 1e-3;
  // [m] distance threshold to consider reaching a goal
  static inline constexpr double kReachedDistanceThreshold = 0.05;
  // direction similarity threshold to consider moving toward a goal
  //   (cos(60 degrees) = 0.5)
  static inline constexpr double kDirectionSimilarityThreshold = 0.5;

  IntentInference(const double& transition_probability = 0.1,
                  const double& confidence_lower_bound = 0.3,
                  const double& confidence_upper_bound = 0.9);

  void setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects);

  inline void setGripperPosition(const geometry_msgs::Point& position) {
    gripper_position_ = position;
  }
  
  inline void setUserCommandVelocity(const geometry_msgs::Vector3& velocity) {
    user_command_velocity_ = velocity;
  }

  inline void setGripperStatus(const std_msgs::Bool& status) {
    gripper_status_ = status;
  }

  inline void setSafetyButtonStatus(const bool& is_pressed) {
    is_positional_safety_button_pressed_ = is_pressed;
  }

  inline detection_msgs::DetectedObject getMostLikelyGoal() const {
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

  inline detection_msgs::ObjectBeliefMap getBelief() const {
    return belief_;
  }

  inline GripperMotionState getGripperMotionState() const {
    return gripper_motion_state_;
  }
  
  inline const bool isLocked() const {
    return gripper_motion_state_ == GripperMotionState::LOCKED;
  }

  inline const bool isReached() const {
    return gripper_motion_state_ == GripperMotionState::REACHED;
  }

  visualization_msgs::MarkerArray getBeliefVisualization();

  geometry_msgs::Vector3 getGoalDirection(const detection_msgs::DetectedObject& goal) const;

  void updateBelief();

 private:
  void updatePositionToBaseLink();
  geometry_msgs::Point transformOdomToBaseLink(const geometry_msgs::Point& point_in_odom);
  void updateGripperMotionState();
  bool isUserCommandIdle() const;
  bool isAnyGoalReached() const;
  bool isAwayFromAllGoals() const;
  
  double getCosineSimilarity(const geometry_msgs::Vector3& user_direction,
                             const geometry_msgs::Vector3& goal_direction) const;
  void normalizeBelief();
  void calculateConfidence();

  double transition_probability_; // [0,1], constant
  double confidence_lower_bound_; // [0,1], constant
  double confidence_upper_bound_; // [0,1], constant
  double confidence_; // [0,1]

  bool is_positional_safety_button_pressed_;
  
  geometry_msgs::Point gripper_position_;
  geometry_msgs::Vector3 user_command_velocity_;
  std_msgs::Bool gripper_status_;
  detection_msgs::DetectedObjectArray recorded_objects_;
  detection_msgs::ObjectBeliefMap belief_;
  detection_msgs::ObjectPositionMap position_to_base_link_;
  Tf2ListenerWrapper tf2_listener_;
  GripperMotionState gripper_motion_state_;
  GripperMotionState last_gripper_motion_state_;
};

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
