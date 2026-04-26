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
  PICK_MANUAL = 1,
  PICK_ASSIST = 2,
  PICK_REJECT = 3,
  PICK_READY = 4,
  PICK_GRASP = 5,
  RETURN_ASSIST = 6
};

static std::string toString(RobotState s) {
  switch(s) {
    case RobotState::STANDBY:      return "STANDBY";
    case RobotState::PICK_MANUAL:  return "PICK_MANUAL";
    case RobotState::PICK_ASSIST:  return "PICK_ASSIST";
    case RobotState::PICK_REJECT:  return "PICK_REJECT";
    case RobotState::PICK_READY:   return "PICK_READY";
    case RobotState::PICK_GRASP:    return "PICK_GRASP";
    case RobotState::RETURN_ASSIST: return "RETURN_ASSIST";
  }
  return "UNKNOWN"; // fallback
}

class IntentInference {
 public:
  using ResetPoseHandler = std::function<bool()>;

  // numerical tolerances (not tunable — keep as compile-time constants)
  static inline constexpr double kDirectionTolerance = 1e-3;  // [m]
  static inline constexpr double kDistanceTolerance  = 1e-3;  // [m]

  IntentInference();

  static void loadPoseParam(const ros::NodeHandle& nh,
                            const std::string& prefix,
                            geometry_msgs::Pose& pose);

  void parseParameters(const ros::NodeHandle& nh);

  inline void registerResetPoseHandler(const ResetPoseHandler& handler) {
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

  void setExperimentID(const int& id);

  detection_msgs::DetectedObjectArray getObjectsWithBelief() const;

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

  inline const bool isReturnAssistanceActive() const {
    return robot_state_ == RobotState::RETURN_ASSIST;
  }

  inline geometry_msgs::Pose getStagingPose() const {
    return staging_pose_;
  }

  void updateObjectPositionToTmBase();
  
  void updateGripperPosition();

  /**
   * @note The procedure and priority definition of the state machine are
   *   listed in `marslite_control/README.md`
   */
  void updateRobotState();

  void triggerResetPoseService();

  void updateBelief();

 private:

  void removeTargetFromRecordedObjects();

  /**
   * @brief Find and remove the recorded object nearest to the gripper.
   *        Used when the gripper closes to handle cases where (a) camera
   *        error shifts the target position, or (b) the operator grasps
   *        a different object than the highest-belief target.
   * @return true if an object was removed.
   */
  bool removeNearestGraspedObject();
  
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
    return confidence_ >= confidence_threshold_;
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

  const bool isTowardStagingPose() const;

  const bool isNearStagingPose() const;

  const bool isAwayFromStagingPose() const;

  geometry_msgs::Vector3 getGoalDirection(const detection_msgs::DetectedObject& goal) const;
  
  double getCosineSimilarity(const geometry_msgs::Vector3& user_direction,
                             const geometry_msgs::Vector3& goal_direction) const;

  void normalizeBelief();

  void calculateConfidence();

  static inline double norm(const geometry_msgs::Vector3& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  }

  // --- Parameters (loaded from YAML via parseParameters) ---
  double transition_probability_;
  double direction_likelihood_parameter_;
  double proximity_likelihood_parameter_;
  double confidence_threshold_;
  double z_distance_threshold_;
  double near_distance_threshold_;
  double position_tolerance_;
  double pick_area_distance_threshold_;
  double grasp_removal_distance_threshold_;
  double user_command_speed_tolerance_;
  double toward_target_similarity_;
  double away_target_similarity_;
  double assist_dwell_time_;
  double reject_cooldown_time_;
  geometry_msgs::Pose staging_pose_left_;
  geometry_msgs::Pose staging_pose_front_;
  geometry_msgs::Pose staging_pose_exp3_;
  double staging_pose_reached_angle_;   // [rad] azimuth convergence threshold
  double confidence_; // [0,1], computed at runtime

  bool use_sim_;
  bool return_assist_enabled_;
  bool is_inference_activated_;
  bool is_positional_safety_button_pressed_;
  bool is_assist_dwell_timer_started_;
  bool is_reject_cooldown_timer_started_;
  bool is_reset_pose_signal_transferred_;
  bool is_reset_pose_completed_;
  int experiment_id_;

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
  bool grasp_completed_;       // latches true when gripper closes; cleared on RETURN_ASSIST exit
  bool return_in_progress_;    // latches true on RETURN_ASSIST entry; enables auto-resume from STANDBY
  ResetPoseHandler reset_pose_handler_;
  geometry_msgs::Pose staging_pose_;
};

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
