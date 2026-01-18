#ifndef MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
#define MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H

#include <functional>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <map>

#include "detection_msgs/DetectedObject.h"
#include "detection_msgs/DetectedObjectArray.h"
#include "utils/tf2_listener_wrapper.h"

namespace detection_msgs {

struct DetectedObjectComparator {
  inline const bool operator()(
      const detection_msgs::DetectedObject& a,
      const detection_msgs::DetectedObject& b) const {
    return a.centroid.x < b.centroid.x;
  }
};

using ObjectBeliefMap = std::map<detection_msgs::DetectedObject, double, DetectedObjectComparator>;
using ObjectPositionMap = std::map<detection_msgs::DetectedObject, geometry_msgs::PointStamped, DetectedObjectComparator>;

} // namespace detection_msgs


enum class RobotState : uint8_t {
  STANDBY = 0,
  // PICK PHASE (Gripper OPEN) 
  PICK_MANUAL = 1,
  PICK_ASSIST = 2,
  PICK_REJECT = 3,
  PICK_READY = 4,
  PICK_GRASP = 5
  /// TODO: Add PLACE phase states
  // PLACE_MANUAL = 6,
  // PLACE_ASSIST = 7,
  // PLACE_READY = 8,
  // PLACE_COMPLETE = 9,
};

static std::string toString(RobotState s) {
  switch(s) {
    case RobotState::STANDBY:      return "STANDBY";
    case RobotState::PICK_MANUAL:  return "PICK_MANUAL";
    case RobotState::PICK_ASSIST:  return "PICK_ASSIST";
    case RobotState::PICK_REJECT:  return "PICK_REJECT";
    case RobotState::PICK_READY:   return "PICK_READY";
    case RobotState::PICK_GRASP:   return "PICK_GRASP";
    // case RobotState::PLACE_MANUAL:  return "PLACE_MANUAL";
    // case RobotState::PLACE_ASSIST:  return "PLACE_ASSIST";
    // case RobotState::PLACE_READY:   return "PLACE_READY";
    // case RobotState::PLACE_COMPLETE: return "PLACE_COMPLETE";
  }
  return "UNKNOWN"; // fallback
}

class IntentInference {
 public:
  using ResetPoseHandler = std::function<bool()>;

  static inline constexpr double kDirectionLikelihoodParameter = 4.0;
  static inline constexpr double kProximityLikelihoodParameter = 3.0;
  static inline constexpr double kAssistanceActivatedConfidenceThreshold = 2;
  // [m] Used for isNearTarget() and isTargetReached()
  static inline constexpr double kZDistanceThreshold = 0.15;
  // [m] Ready-to-pick zone
  static inline constexpr double kNearDistanceThreshold = 0.02;
  // [m] 
  static inline constexpr double kPositionTolerance = 5e-3;
  // [m]
  static inline constexpr double kPickAreaDistanceThreshold = 0.40;
  // [m]
  static inline constexpr double kDirectionTolerance = 1e-3;
  // [m/s]
  static inline constexpr double kUserCommandSpeedTolerance = 0.15;
  // [rad] cos(45 degrees) = 0.707
  static inline constexpr double kTowardTargetDirectionSimilarityThreshold = 0.707;
  // [rad] cos(135 degrees) = -0.707
  static inline constexpr double kAwayTargetDirectionSimilarityThreshold = -0.707;
  // [s] 
  static inline constexpr double kAssistDwellTime = 0.3;
  // [s] 
  static inline constexpr double kRejectCooldownTime = 1.0;

  IntentInference(const double& transition_probability = 0.1);

  void registerResetPoseHandler(const ResetPoseHandler& handler) {
    reset_pose_handler_ = handler;
  }

  inline void setUseSim(const bool& use_sim) {
    use_sim_ = use_sim;
  }

  void setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects);
  
  inline void setUserCommandVelocity(const geometry_msgs::Vector3& velocity) {
    user_command_velocity_ = velocity;
  }

  inline void setGripperStatus(const std_msgs::Bool& status) {
    gripper_status_ = status;
  }

  inline void setSafetyButtonStatus(const bool& is_pressed) {
    is_positional_safety_button_pressed_ = is_pressed;
  }

  inline detection_msgs::DetectedObjectArray getObjectsWithBelief() const {
    detection_msgs::DetectedObjectArray objects_with_belief;
    for (const auto& [object, prob] : belief_) {
      objects_with_belief.objects.push_back(object);
      objects_with_belief.objects.back().frame = "tm_base";
      objects_with_belief.objects.back().centroid = position_wrt_tm_base_.at(object).point;
      objects_with_belief.objects.back().confidence = prob;
    }
    return objects_with_belief;
  }

  inline RobotState getRobotState() const {
    return robot_state_;
  }

  inline geometry_msgs::Point getGripperPosition() const {
    return gripper_position_;
  }

  inline detection_msgs::DetectedObject getTargetObject() const {
    if (belief_.empty())
      return detection_msgs::DetectedObject();
    
    const auto max_it = std::max_element(
        belief_.begin(), belief_.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second;
        }
    );
    return max_it->first;
  }

  inline geometry_msgs::Point getTargetPosition() const {
    if (belief_.empty())
      return geometry_msgs::Point();
    return position_wrt_tm_base_.at(this->getTargetObject()).point;
  }

  inline geometry_msgs::Vector3 getTargetDirection() const {
    if (belief_.empty())
      return geometry_msgs::Vector3();
    return this->getGoalDirection(this->getTargetObject());
  }

  inline detection_msgs::ObjectBeliefMap getBelief() const {
    return belief_;
  }

  visualization_msgs::MarkerArray getBeliefVisualization();

  inline const bool isPickAssistanceActive() const {
    return robot_state_ == RobotState::PICK_ASSIST;
  }

  void updateObjectPositionToTmBase();
  
  void updateGripperPosition();

  /**
   * @note The procedure and priority definition of the state machine are
   *   listed in `marslite_control/README.md`
   */
  void updateRobotState();

  void updateBelief();

 private:

  void removeTargetFromRecordedObjects();
  
  /**
   * @note `gripper_motion_state_` set to `PICK_READY` if true.
   */
  const bool isNearTarget() const;

  /**
   * @note This function was designed to avoid `gripper_motion_state_` being 
   *   transferred from `PICK_ASSIST` to `PICK_READY` when the gripper immediately
   *   enters the `PICK_READY` zone (sphere with radius `kNearDistanceThreshold`).
   *   The transfer will be done as the gripper actually reaches the centroid
   *   of the detected object.
   */
  const bool isTargetReached() const;

  const bool isInPickArea() const;

  /**
   * @note This is one of the conditions that `gripper_motion_state_` is
   *   transferred from `PICK_MANUAL` to `PICK_ASSIST`.
   */
  const bool isTowardTarget() const;

  inline const bool isConfidenceHighEnough() const {
    return confidence_ >= kAssistanceActivatedConfidenceThreshold;
  }

  /**
   * @brief Trigger `assist_dwell_timer_`.
   * @note `assist_dwell_timer_` is used for `PICK_MANUAL` -> `PICK_ASSIST`
   *    transfer.
   */
  void triggerAssistDwellTimer();

  /**
   * @return true if `assist_dwell_timer_` counts over `isAssistDwellTimePassed`
   *    seconds.
   * @note `gripper_motion_state_` is transferred from `PICK_MANUAL` to `PICK_ASSIST`
   *    if true.
   */
  const bool isAssistDwellTimePassed();

  /**
   * @note `gripper_motion_state_` is transferred from `PICK_ASSIST` to
   *    `PICK_REJECT` if true.
   */
  const bool isAwayFromTarget() const;

  /**
   * @brief Trigger `cooldown_timer_`.
   * @note `cooldown_timer_` is used for `PICK_REJECT` -> `PICK_MANUAL` transfer.
   */
  void triggerRejectCooldownTimer();
  
  /**
   * @return true if `cooldown_timer_` counts over `kRejectCooldownTime` seconds.
   * @note `gripper_motion_state_` is transferred from `PICK_REJECT` to
   *    `PICK_MANUAL` if true.
   */
  const bool isRejectCooldownTimePassed();

  geometry_msgs::Vector3 getGoalDirection(const detection_msgs::DetectedObject& goal) const;
  
  double getCosineSimilarity(const geometry_msgs::Vector3& user_direction,
                             const geometry_msgs::Vector3& goal_direction) const;

  void normalizeBelief();

  void calculateConfidence();

  static inline double norm(const geometry_msgs::Vector3& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  }

  double transition_probability_; // [0,1], constant
  double confidence_; // [0,1]

  bool use_sim_;
  bool is_inference_activated_;
  bool is_positional_safety_button_pressed_;
  bool is_assist_dwell_timer_started_;
  bool is_reject_cooldown_timer_started_;
  bool is_reset_pose_signal_transferred_;
  bool is_reset_pose_completed_;

  struct {
    ros::Time start_time;
    ros::Time current_time;
    double getTimeDifference() const {
      return (current_time - start_time).toSec();
    }
  } assist_dwell_timer_, cooldown_timer_; 
  geometry_msgs::Point gripper_position_;
  geometry_msgs::Vector3 user_command_velocity_;
  std_msgs::Bool gripper_status_;
  detection_msgs::DetectedObjectArray recorded_objects_;
  detection_msgs::ObjectBeliefMap belief_;
  detection_msgs::ObjectPositionMap position_wrt_odom_;
  detection_msgs::ObjectPositionMap position_wrt_tm_base_;
  Tf2ListenerWrapper tf2_listener_;
  RobotState robot_state_;
  ResetPoseHandler reset_pose_handler_;
};

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
