#include "shared_control/intent_inference.h"
#include <ros/time.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

IntentInference::IntentInference() :
    // Defaults (overridden by parseParameters if YAML is loaded)
    transition_probability_(0.1),
    direction_likelihood_parameter_(4.0),
    proximity_likelihood_parameter_(3.0),
    confidence_threshold_(0.45),
    z_distance_threshold_(0.15),
    near_distance_threshold_(0.03),
    position_tolerance_(5e-3),
    pick_area_distance_threshold_(0.40),
    grasp_removal_distance_threshold_(0.10),
    user_command_speed_tolerance_(0.15),
    toward_target_similarity_(0.5),
    away_target_similarity_(-0.707),
    assist_dwell_time_(0.3),
    reject_cooldown_time_(1.0),
    staging_pose_reached_angle_(0.05),  // ~3°
    confidence_(0.0),
    use_sim_(false),
    return_assist_enabled_(true),
    is_inference_activated_(false),
    is_positional_safety_button_pressed_(false),
    is_assist_dwell_timer_started_(false),
    is_reject_cooldown_timer_started_(false),
    is_reset_pose_signal_transferred_(false),
    is_reset_pose_completed_(false),
    robot_state_(RobotState::STANDBY),
    grasp_completed_(false),
    return_in_progress_(false) {
  belief_ = {};
  position_wrt_odom_ = {};
  position_wrt_tm_base_ = {};
}

void IntentInference::loadPoseParam(const ros::NodeHandle& nh,
                                    const std::string& prefix,
                                    geometry_msgs::Pose& pose) {
  nh.getParam(prefix + "/position/x",    pose.position.x);
  nh.getParam(prefix + "/position/y",    pose.position.y);
  nh.getParam(prefix + "/position/z",    pose.position.z);
  nh.getParam(prefix + "/orientation/x", pose.orientation.x);
  nh.getParam(prefix + "/orientation/y", pose.orientation.y);
  nh.getParam(prefix + "/orientation/z", pose.orientation.z);
  nh.getParam(prefix + "/orientation/w", pose.orientation.w);
}

void IntentInference::parseParameters(const ros::NodeHandle& nh) {
  loadPoseParam(nh, "staging_pose_left", staging_pose_left_);
  loadPoseParam(nh, "staging_pose_front", staging_pose_front_);
  loadPoseParam(nh, "staging_pose_exp3", staging_pose_exp3_);

  nh.getParam("intent_inference/transition_probability", transition_probability_);
  nh.getParam("intent_inference/direction_likelihood_parameter", direction_likelihood_parameter_);
  nh.getParam("intent_inference/proximity_likelihood_parameter", proximity_likelihood_parameter_);
  nh.getParam("intent_inference/confidence_threshold", confidence_threshold_);
  nh.getParam("intent_inference/z_distance_threshold", z_distance_threshold_);
  nh.getParam("intent_inference/near_distance_threshold", near_distance_threshold_);
  nh.getParam("intent_inference/position_tolerance", position_tolerance_);
  nh.getParam("intent_inference/pick_area_distance_threshold", pick_area_distance_threshold_);
  nh.getParam("intent_inference/grasp_removal_distance_threshold", grasp_removal_distance_threshold_);
  nh.getParam("intent_inference/user_command_speed_tolerance", user_command_speed_tolerance_);
  nh.getParam("intent_inference/toward_target_similarity", toward_target_similarity_);
  nh.getParam("intent_inference/away_target_similarity", away_target_similarity_);
  nh.getParam("intent_inference/assist_dwell_time", assist_dwell_time_);
  nh.getParam("intent_inference/reject_cooldown_time", reject_cooldown_time_);
  nh.getParam("intent_inference/staging_pose_reached_angle", staging_pose_reached_angle_);
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void IntentInference::setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects) {
  recorded_objects_ = {};
  belief_ = {};
  position_wrt_odom_ = {};
  position_wrt_tm_base_ = {};

  const size_t n = recorded_objects.objects.size();
  if (n > 0) {
    for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
      geometry_msgs::PointStamped centroid;
      centroid.header.frame_id = obj.frame;
      centroid.point = obj.centroid;
      recorded_objects_.objects.push_back(obj);
      position_wrt_odom_[obj] = tf2_listener_.transformData(centroid, "odom");
      position_wrt_tm_base_[obj] = tf2_listener_.transformData(centroid, "tm_base");
      belief_[obj] = 1.0 / static_cast<double>(n);
    }
  }
}

void IntentInference::setExperimentID(const int& id) {
  experiment_id_ = id;
  switch (experiment_id_) {
    case 1:
      ROS_WARN_STREAM_ONCE("No staging pose specified for Experiment 1! "\
          << "Return empty Pose() instead...");
      staging_pose_ = geometry_msgs::Pose();
    break;
    case 2:
      staging_pose_ = staging_pose_left_;
    break;
    case 3:
      staging_pose_ = staging_pose_exp3_;
    break;
    default: break;
  }
  return_assist_enabled_ = (experiment_id_ != 1);
}

detection_msgs::DetectedObjectArray IntentInference::getObjectsWithBelief() const {
  detection_msgs::DetectedObjectArray objects_with_belief;
  for (const auto& [object, prob] : belief_) {
    objects_with_belief.objects.push_back(object);
    objects_with_belief.objects.back().frame = "tm_base";
    objects_with_belief.objects.back().centroid = position_wrt_tm_base_.at(object).point;
    objects_with_belief.objects.back().confidence = prob;
  }
  return objects_with_belief;
}

visualization_msgs::MarkerArray IntentInference::getBeliefVisualization() {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& [object, belief_value] : belief_) {
    // object marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "tm_base";
    marker.header.stamp = ros::Time::now();
    marker.ns = "intent_belief";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = position_wrt_tm_base_[object].point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0 - belief_value;
    marker.color.g = belief_value;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = ros::Duration(1.0);
    marker_array.markers.push_back(marker);

    // text marker
    visualization_msgs::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "intent_belief_text";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = position_wrt_tm_base_[object].point;
    text_marker.pose.position.z += 0.15;
    text_marker.scale.z = 0.06;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << belief_value;
    text_marker.text = oss.str();
    text_marker.lifetime = ros::Duration(1.0);
    marker_array.markers.push_back(text_marker);
  }

  // `robot_state` marker
  visualization_msgs::Marker gripper_marker;
  gripper_marker.header.frame_id = "tm_base";
  gripper_marker.header.stamp = ros::Time::now();
  gripper_marker.ns = "robot_state";
  gripper_marker.id = id++;
  gripper_marker.type = visualization_msgs::Marker::SPHERE;
  gripper_marker.action = visualization_msgs::Marker::ADD;
  gripper_marker.pose.position.x = gripper_position_.x;
  gripper_marker.pose.position.y = gripper_position_.y;
  gripper_marker.pose.position.z = gripper_position_.z;
  gripper_marker.pose.orientation.w = 1.0;
  gripper_marker.scale.x = 0.5;
  gripper_marker.scale.y = 0.5;
  gripper_marker.scale.z = 0.5;
  gripper_marker.color.a = 0.1;
  gripper_marker.lifetime = ros::Duration(1.0);

  switch (robot_state_) {
    case RobotState::PICK_MANUAL:
      // light green
      gripper_marker.color.r = 0.6;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.6;
    break;
    case RobotState::PICK_ASSIST:
      // green
      gripper_marker.color.r = 0.0;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.0;
    break;
    case RobotState::PICK_REJECT:
      // red
      gripper_marker.color.r = 1.0;
      gripper_marker.color.g = 0.0;
      gripper_marker.color.b = 0.0;
    break;
    case RobotState::PICK_READY:
      // yellow
      gripper_marker.color.r = 1.0;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.0;
    break;
    case RobotState::PICK_GRASP:
      // blue
      gripper_marker.color.r = 0.0;
      gripper_marker.color.g = 0.0;
      gripper_marker.color.b = 1.0;
    break;
    case RobotState::RETURN_ASSIST:
      // cyan
      gripper_marker.color.r = 0.0;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 1.0;
    break;
    case RobotState::STANDBY:
    default:
      // gray
      gripper_marker.color.r = 0.5;
      gripper_marker.color.g = 0.5;
      gripper_marker.color.b = 0.5;
    break;
  }
  marker_array.markers.push_back(gripper_marker);

  return marker_array;
}

void IntentInference::updateObjectPositionToTmBase() {
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    position_wrt_tm_base_[recorded_object] = tf2_listener_.transformData(
        position_wrt_odom_[recorded_object], "tm_base"
    );
  }
}

void IntentInference::updateGripperPosition() {
  const std::string source_frame = use_sim_ ? "robotiq_85_base_link" : "tm_gripper";
  gripper_position_ = tf2_listener_.lookupTransform<geometry_msgs::Point>("tm_base", source_frame);
}

void IntentInference::updateRobotState() {
  if (!is_positional_safety_button_pressed_) {
    // ----- Priority 1: safety button not pressed (STANDBY) -----
    robot_state_ = RobotState::STANDBY;
  } else if (gripper_status_.data) {
    // ----- Priority 2: gripper is closed (PICK_GRASP) -----
    if (robot_state_ != RobotState::PICK_GRASP) {
      // Only remove on the transition into PICK_GRASP (first tick)
      this->removeNearestGraspedObject();
    }
    robot_state_ = RobotState::PICK_GRASP;
    grasp_completed_ = true;  // latch: an object has been grasped
  } else if (this->isNearTarget()) {
    // ----- Priority 3: gripper is near target (PICK_READY) -----
    if (robot_state_ == RobotState::PICK_ASSIST && !this->isTargetReached()) {
      // [Exception] remain PICK_ASSIST if the planned position has not been reached
      robot_state_ = RobotState::PICK_ASSIST;
    } else {
      this->triggerResetPoseService();
      if (is_reset_pose_completed_) {
        robot_state_ = RobotState::PICK_READY;
      }
    }
  } else {
    // ----- Finite State Machine -----
    switch (robot_state_) {
      case RobotState::STANDBY:
        if (return_assist_enabled_ && return_in_progress_) {
          // Resume RETURN_ASSIST state if the assistance is still in progress
          robot_state_ = RobotState::RETURN_ASSIST;
        } else if (is_reject_cooldown_timer_started_) {
          // Resume PICK_REJECT state if the reject cooldown timer is still active
          robot_state_ = RobotState::PICK_REJECT;
        } else {
          robot_state_ = RobotState::PICK_MANUAL;
        }
      break;
      case RobotState::PICK_GRASP:
        // --- No state transfer happens here ---
      break;
      case RobotState::PICK_MANUAL:
        if (return_assist_enabled_ && grasp_completed_ && this->isTowardStagingPose()) {
          // --- Return-assist check ---
          robot_state_ = RobotState::RETURN_ASSIST;
          return_in_progress_ = true;
        } else if (this->isInPickArea() && this->isTowardTarget() && this->isConfidenceHighEnough()) {
          // --- Pick-assist check ---
          if (!is_assist_dwell_timer_started_) {
            this->triggerAssistDwellTimer();
            is_assist_dwell_timer_started_ = true;
          }

          if (this->isAssistDwellTimePassed()) {
            robot_state_ = RobotState::PICK_ASSIST;
            is_assist_dwell_timer_started_ = false;
            grasp_completed_ = false;  // starting a new pick cycle
          }
        } else if (this->isAwayFromTarget()) {
          // remain PICK_MANUAL
          is_assist_dwell_timer_started_ = false;
        }
        is_reset_pose_signal_transferred_ = false;
        is_reset_pose_completed_ = false;
      break;
      case RobotState::PICK_ASSIST:
        if (this->isAwayFromTarget()) {
          robot_state_ = RobotState::PICK_REJECT;
        }
        // else: remain PICK_ASSIST
      break;
      case RobotState::PICK_REJECT:
        if (!is_reject_cooldown_timer_started_) {
          this->triggerRejectCooldownTimer();
          is_reject_cooldown_timer_started_ = true;
        }

        if (this->isRejectCooldownTimePassed()) {
          // Transfer to PICK_MANUAL after cooldown
          robot_state_ = RobotState::PICK_MANUAL;
          is_reject_cooldown_timer_started_ = false;
        }
        // else: remain PICK_REJECT
      break;
      case RobotState::RETURN_ASSIST:
        ROS_INFO_STREAM_THROTTLE(10, "Return assist activated.");
        if (is_reset_pose_signal_transferred_ && is_reset_pose_completed_) {
          // Tick N+1: Operate state transfer
          ROS_INFO_STREAM("Return assist deactivated.");
          robot_state_ = RobotState::PICK_MANUAL;
          grasp_completed_ = false;
          return_in_progress_ = false;
        } else if (this->isNearStagingPose() || this->isAwayFromStagingPose()) {
          // Tick N: Trigger reset pose service
          this->triggerResetPoseService();
        }
        // else: remain RETURN_ASSIST
      break;
      case RobotState::PICK_READY:
        // Not near the target anymore -> transfer back to PICK_MANUAL
        robot_state_ = RobotState::PICK_MANUAL;
      break;
      default: break;
    }
    
  }
}

void IntentInference::triggerResetPoseService() {
  if (!is_reset_pose_signal_transferred_) {
    if (reset_pose_handler_) {
      is_reset_pose_completed_ = reset_pose_handler_();
      is_reset_pose_signal_transferred_ = true;
    } else {
      ROS_WARN("reset_pose_handler_");
    }
  }
}

void IntentInference::updateBelief() {
  if (robot_state_ != RobotState::PICK_MANUAL || recorded_objects_.objects.empty())  return;

  // Update belief using Bayesian inference with Markov transition model
  detection_msgs::ObjectBeliefMap new_belief;
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    geometry_msgs::Vector3 goal_direction = this->getGoalDirection(recorded_object);

    // (1) direction_likelihood
    const double direction_similarity = getCosineSimilarity(user_command_velocity_, goal_direction);
    const double direction_base_score = (direction_similarity + 1) / 2;  // [-1, 1] -> [0, 1]
    const double direction_likelihood = std::pow(direction_base_score, direction_likelihood_parameter_);

    // (2) proximity likelihood
    const double goal_distance = std::sqrt(
        goal_direction.x * goal_direction.x +
        goal_direction.y * goal_direction.y +
        goal_direction.z * goal_direction.z
    );
    const double proximity_likelihood = std::exp(-proximity_likelihood_parameter_ * goal_distance);

    // Markov transition update
    const double likelihood = direction_likelihood * proximity_likelihood;
    double sum_transitions = 0.0;
    const size_t n = belief_.size();
    for (const auto& [object, probability] : belief_) {
      double transition;
      if (n == 1) {
        transition = 1.0;
      } else if (recorded_object == object) {
        transition = 1.0 - transition_probability_;
      } else {
        transition = transition_probability_ / static_cast<double>(n - 1);
      }
      sum_transitions += transition * probability;
    }
    new_belief[recorded_object] = likelihood * sum_transitions;
  }
  belief_ = new_belief;

  this->normalizeBelief();
  this->calculateConfidence();
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void IntentInference::removeTargetFromRecordedObjects() {
  const detection_msgs::DetectedObject target = this->getTargetObject();
  if (target == detection_msgs::DetectedObject())
    return;
  recorded_objects_.objects.erase(
      std::remove_if(
          recorded_objects_.objects.begin(),
          recorded_objects_.objects.end(),
          [&target](const detection_msgs::DetectedObject& obj) {
              return obj.label == target.label && obj.frame == target.frame
                  && obj.centroid == target.centroid;
          }
      ),
      recorded_objects_.objects.end()
  );
  belief_.erase(target);
  position_wrt_odom_.erase(target);
  position_wrt_tm_base_.erase(target);

  if (!recorded_objects_.objects.empty()) {
    for (const detection_msgs::DetectedObject& obj : recorded_objects_.objects) {
      belief_[obj] = 1.0 / recorded_objects_.objects.size();
    }
  }
}

bool IntentInference::removeNearestGraspedObject() {
  if (recorded_objects_.objects.empty()) return false;

  // Find the recorded object closest to the gripper
  double min_dist = std::numeric_limits<double>::max();
  const detection_msgs::DetectedObject* nearest = nullptr;
  for (const auto& obj : recorded_objects_.objects) {
    const auto it = position_wrt_tm_base_.find(obj);
    if (it == position_wrt_tm_base_.end()) continue;
    const double dx = it->second.point.x - gripper_position_.x;
    const double dy = it->second.point.y - gripper_position_.y;
    const double dz = it->second.point.z - gripper_position_.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = &obj;
    }
  }

  if (!nearest || min_dist > grasp_removal_distance_threshold_) return false;

  // Remove the nearest object
  const detection_msgs::DetectedObject to_remove = *nearest;
  recorded_objects_.objects.erase(
      std::remove_if(
          recorded_objects_.objects.begin(),
          recorded_objects_.objects.end(),
          [&to_remove](const detection_msgs::DetectedObject& obj) {
              return obj.label == to_remove.label && obj.frame == to_remove.frame
                  && obj.centroid == to_remove.centroid;
          }
      ),
      recorded_objects_.objects.end()
  );
  belief_.erase(to_remove);
  position_wrt_odom_.erase(to_remove);
  position_wrt_tm_base_.erase(to_remove);

  if (!recorded_objects_.objects.empty()) {
    for (const detection_msgs::DetectedObject& obj : recorded_objects_.objects) {
      belief_[obj] = 1.0 / recorded_objects_.objects.size();
    }
  }
  return true;
}

const bool IntentInference::isNearTarget() const {
  if (belief_.empty())  return false;
  const geometry_msgs::Vector3 target_direction = this->getTargetDirection();
  const double target_distance_xy = std::hypot(target_direction.x, target_direction.y);
  return target_distance_xy < near_distance_threshold_
      && std::abs(target_direction.z) < z_distance_threshold_;
}

const bool IntentInference::isTargetReached() const {
  if (belief_.empty())  return false;
  const geometry_msgs::Vector3 target_direction = this->getTargetDirection();
  const double target_distance_xy = std::hypot(target_direction.x, target_direction.y);
  return target_distance_xy < position_tolerance_
      && std::abs(target_direction.z) < z_distance_threshold_;
}

const bool IntentInference::isInPickArea() const {
  if (belief_.empty())  return false;

  const detection_msgs::DetectedObject target = this->getTargetObject();
  const auto it = position_wrt_tm_base_.find(target);
  if (it == position_wrt_tm_base_.end())  return false;
  
  const double dx = it->second.point.x - gripper_position_.x;
  const double dy = it->second.point.y - gripper_position_.y;
  const double dz = it->second.point.z - gripper_position_.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) < pick_area_distance_threshold_;
}

const bool IntentInference::isTowardTarget() const {
  const geometry_msgs::Vector3 target_direction = this->getTargetDirection();
  const double target_distance = norm(target_direction);
  if (target_distance < kDirectionTolerance)  return false;  // avoid divide-by-zero
  
  const double direction_similarity = this->getCosineSimilarity(
      user_command_velocity_,
      target_direction
  );
  return direction_similarity >= toward_target_similarity_;
}

void IntentInference::triggerAssistDwellTimer() {
  assist_dwell_timer_.start_time = assist_dwell_timer_.current_time = ros::Time::now();
}

const bool IntentInference::isAssistDwellTimePassed() {
  assist_dwell_timer_.current_time = ros::Time::now();
  return assist_dwell_timer_.getTimeDifference() >= assist_dwell_time_;
}

const bool IntentInference::isAwayFromTarget() const {
  const geometry_msgs::Vector3 target_direction = this->getTargetDirection();
  const double target_distance = norm(target_direction);
  if (target_distance < kDirectionTolerance)  return false;  // avoid divide-by-zero

  const double user_command_speed = norm(user_command_velocity_);
  if (user_command_speed < user_command_speed_tolerance_)  return false; // ignore little speed

  const double direction_similarity = this->getCosineSimilarity(
      user_command_velocity_,
      target_direction
  );
  return direction_similarity < away_target_similarity_;
}

void IntentInference::triggerRejectCooldownTimer() {
  cooldown_timer_.start_time = cooldown_timer_.current_time = ros::Time::now();
}

const bool IntentInference::isRejectCooldownTimePassed() {
  cooldown_timer_.current_time = ros::Time::now();
  return cooldown_timer_.getTimeDifference() >= reject_cooldown_time_;
}

const bool IntentInference::isTowardStagingPose() const {
  if (experiment_id_ == 1)  return false;

  geometry_msgs::Vector3 staging_direction;
  staging_direction.x = staging_pose_.position.x - gripper_position_.x;
  staging_direction.y = staging_pose_.position.y - gripper_position_.y;
  staging_direction.z = staging_pose_.position.z - gripper_position_.z;
  const double dist = norm(staging_direction);
  if (dist < kDirectionTolerance) return false;

  const double user_speed = norm(user_command_velocity_);
  if (user_speed < user_command_speed_tolerance_) return false;

  return this->getCosineSimilarity(user_command_velocity_, staging_direction)
      >= toward_target_similarity_;
}

const bool IntentInference::isNearStagingPose() const {
  if (experiment_id_ == 1)  return false;

  const double theta_cur = std::atan2(gripper_position_.y, gripper_position_.x);
  const double theta_stg = std::atan2(staging_pose_.position.y,
                                      staging_pose_.position.x);
  double dtheta = theta_stg - theta_cur;
  if (dtheta >  M_PI) dtheta -= 2.0 * M_PI;
  if (dtheta < -M_PI) dtheta += 2.0 * M_PI;
  return std::abs(dtheta) < staging_pose_reached_angle_;
}

const bool IntentInference::isAwayFromStagingPose() const {
  if (experiment_id_ == 1)  return false;

  geometry_msgs::Vector3 staging_direction;
  staging_direction.x = staging_pose_.position.x - gripper_position_.x;
  staging_direction.y = staging_pose_.position.y - gripper_position_.y;
  staging_direction.z = staging_pose_.position.z - gripper_position_.z;
  const double dist = norm(staging_direction);
  if (dist < kDirectionTolerance) return false;

  const double user_speed = norm(user_command_velocity_);
  if (user_speed < user_command_speed_tolerance_) return false;

  return this->getCosineSimilarity(user_command_velocity_, staging_direction)
      < away_target_similarity_;
}

geometry_msgs::Vector3 IntentInference::getGoalDirection(const detection_msgs::DetectedObject& goal) const {
  geometry_msgs::Vector3 goal_direction;
  goal_direction.x = position_wrt_tm_base_.at(goal).point.x - gripper_position_.x;
  goal_direction.y = position_wrt_tm_base_.at(goal).point.y - gripper_position_.y;
  goal_direction.z = position_wrt_tm_base_.at(goal).point.z - gripper_position_.z;
  return goal_direction;
}

double IntentInference::getCosineSimilarity(
    const geometry_msgs::Vector3& user_direction,
    const geometry_msgs::Vector3& goal_direction) const {
  const double dot = user_direction.x * goal_direction.x
      + user_direction.y * goal_direction.y
      + user_direction.z * goal_direction.z;
  const double mag_user = std::sqrt(user_direction.x * user_direction.x
      + user_direction.y * user_direction.y
      + user_direction.z * user_direction.z
  );
  const double mag_goal = std::sqrt(goal_direction.x * goal_direction.x
      + goal_direction.y * goal_direction.y
      + goal_direction.z * goal_direction.z
  );

  if (mag_user == 0.0 || mag_goal == 0.0) return 0.0;
  return dot / (mag_user * mag_goal); // [-1, 1]
}

void IntentInference::normalizeBelief() {
  double sum = 0.0;
  for (const auto& [_, prob] : belief_) sum += prob;
  for (auto& [_, prob] : belief_) prob /= (sum + 1e-9); // avoid divide-by-zero
}

void IntentInference::calculateConfidence() {
  // Rescaled max-belief: 
  //           confidence = (p_max - 1/N) / (1 - 1/N).
  //
  // Maps chance-level belief (1/N) to 0 and full certainty (1.0) to 1,
  //  uniformly across any N. This avoids the bias of normalized entropy,
  //  which penalizes small N disproportionately (e.g. N=2 needed ~89%
  //  concentration to reach threshold 0.45, vs ~78% for N=5).
  const size_t n = belief_.size();
  if (n <= 1) {
    confidence_ = (n == 1) ? 1.0 : 0.0;
    return;
  }
  double p_max = 0.0;
  for (const auto& [_, prob] : belief_) {
    if (prob > p_max) p_max = prob;
  }
  const double chance = 1.0 / static_cast<double>(n);
  confidence_ = (p_max - chance) / (1.0 - chance);
  if (confidence_ < 0.0) confidence_ = 0.0;
  if (confidence_ > 1.0) confidence_ = 1.0;
}