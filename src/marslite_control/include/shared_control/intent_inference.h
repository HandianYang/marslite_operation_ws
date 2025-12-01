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
  UNLOCKED = 1,
  LOCKED = 2,
  RETREATED = 3,
  REACHED = 4,
  GRASPED = 5
};

static std::string toString(GripperMotionState s) {
  switch(s) {
    case GripperMotionState::IDLE:      return "IDLE";
    case GripperMotionState::UNLOCKED:  return "UNLOCKED";
    case GripperMotionState::LOCKED:    return "LOCKED";
    case GripperMotionState::RETREATED: return "RETREATED";
    case GripperMotionState::REACHED:   return "REACHED";
    case GripperMotionState::GRASPED:   return "GRASPED";
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
  // [m] distance from target object to planned position
  static inline constexpr double kTargetShiftDistance = 0.02;
  // [m] distance threshold to consider reaching a goal
  static inline constexpr double kNearDistanceThreshold = 0.03;
  // direction similarity threshold to consider moving toward one object
  //   (cos(45 degrees) = 0.707)
  static inline constexpr double kLockedSimilarityThreshold = 0.707;
  // direction similarity threshold to consider moving away from one object
  //   (cos(90 degrees) = 0)
  static inline constexpr double kRetreatSimilarityThreshold = 0.0;
  // [s] time to confirm locking a target
  static inline constexpr double kLockedTime = 0.5;
  // [s] cooldown time to transfer from RETREATED to UNLOCKED
  static inline constexpr double kRetreatCooldownTime = 1.0;
  // [m] distance tolerance to determine whether two positions are the same
  static inline constexpr double kPositionTolerance = 1e-3;

  IntentInference(const double& transition_probability = 0.1);

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

  /**
   * @note Be vulnerable to empty belief_
   */
  inline detection_msgs::DetectedObject getMostLikelyGoal() const {
    const auto max_it = std::max_element(belief_.begin(), belief_.end(),
                                        [](const auto& a, const auto& b) {
                                            return a.second < b.second;
                                        });
    return max_it->first;
  }

  inline geometry_msgs::Point getMostLikelyGoalPosition() const {
    return position_to_tm_base_.at(this->getMostLikelyGoal());
  }

  /**
   * @note Be cautious of zero output.
   */
  inline geometry_msgs::Point getPlannedPosition() const {
    return planned_position_;
  }
  
  /**
   * @note Be cautious of zero output.
   */
  inline geometry_msgs::Vector3 getTargetDirection() const {
    return target_direction_;
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
  
  inline const bool isInLockedState() const {
    return gripper_motion_state_ == GripperMotionState::LOCKED;
  }

  visualization_msgs::MarkerArray getBeliefVisualization();

  void updateBelief();

 private:
  void updatePositionToBaseLink();
  
  geometry_msgs::Point transformOdomToTMBase(const geometry_msgs::Point& point_in_odom);

  /**
   * @brief Update `gripper_motion_state_` mainly; update `planned_position_`
   *   and `target_direction_` in calculatePlannedPositionAndTargetDirection()
   * @note The procedure and priority definition of the state machine are
   *   listed in `marslite_control/README.md`
   */
  void updateGripperMotionState();

  /**
   * @brief Update `planned_position_` and `target_direction_`
   * @note This function should be called before updating
   *   `gripper_motion_state_`
   */
  void calculatePlannedPositionAndTargetDirection();
  
  /**
   * @note `gripper_motion_state_` set to `REACHED` if true.
   */
  const bool isNearTarget() const;

  /**
   * @note This function was designed to avoid `gripper_motion_state_` being 
   *   transferred from `LOCKED` to `REACHED` when the gripper immediately
   *   enters the `REACHED` zone (sphere with radius `kNearDistanceThreshold`).
   *   The transfer will be done as the gripper actually reaches
   *   `planned_position_`.
   * 
   *   (p.s.) `planned_position_` is defined as the position a few centimeters in
   *   front of the target object.
   */
  const bool isPlannedPositionReached() const;

  /**
   * @note This is one of the conditions that `gripper_motion_state_` is
   *   transferred from `UNLOCKED` to `LOCKED`
   */
  const bool isTowardTarget() const;

  /**
   * @brief Trigger `locked_timer_`.
   * @note `locked_timer_` is used for `UNLOCKED` -> `LOCKED` transfer.
   */
  void triggerLockedTimer();

  /**
   * @return true if `locked_timer_` counts over `kLockedTime` seconds.
   * @note `gripper_motion_state_` is transferred from `UNLOCKED` to `LOCKED`
   *   if true.
   */
  const bool isLockedTimePassed();

  /**
   * @note `gripper_motion_state_` is transferred from `LOCKED` to `RETREATED`
   *   if true.
   */
  const bool isAwayFromTarget() const;

  /**
   * @brief Trigger `cooldown_timer_`.
   * @note `cooldown_timer_` is used for `RETREATED` -> `UNLOCKED` transfer.
   */
  void triggerCooldownTimer();
  
  /**
   * @return true if `cooldown_timer_` counts over `kRetreatCooldownTime` seconds.
   * @note `gripper_motion_state_` is transferred from `RETREATED` to `UNLOCKED`
   *   if true.
   */
  const bool isCooldownTimePassed();

  inline const bool isEmptyTargetPosition() const {
    return planned_position_ == geometry_msgs::Point();
  }

  inline const bool isEmptyTargetDirection() const {
    return target_direction_ == geometry_msgs::Vector3();
  }
  
  geometry_msgs::Vector3 getGoalDirection(const detection_msgs::DetectedObject& goal) const;
  
  double getCosineSimilarity(const geometry_msgs::Vector3& user_direction,
                             const geometry_msgs::Vector3& goal_direction) const;
  void normalizeBelief();
  void calculateConfidence();

  double transition_probability_; // [0,1], constant
  double confidence_; // [0,1]

  bool is_positional_safety_button_pressed_;
  bool is_locked_timer_started_;
  bool is_cooldown_timer_started_;

  struct {
    ros::Time start_time;
    ros::Time current_time;
    double getTimeDifference() const {
      return (current_time - start_time).toSec();
    }
  } locked_timer_, cooldown_timer_; 
  geometry_msgs::Point gripper_position_;
  geometry_msgs::Point planned_position_;  // defined as target's position ahead a bit distance
  geometry_msgs::Vector3 user_command_velocity_;
  geometry_msgs::Vector3 target_direction_;
  std_msgs::Bool gripper_status_;
  detection_msgs::DetectedObjectArray recorded_objects_;
  detection_msgs::ObjectBeliefMap belief_;
  detection_msgs::ObjectPositionMap position_to_tm_base_;
  Tf2ListenerWrapper tf2_listener_;
  GripperMotionState gripper_motion_state_;
  GripperMotionState last_gripper_motion_state_;
};

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
