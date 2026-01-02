#include "shared_control/intent_inference.h"
#include <ros/time.h>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

IntentInference::IntentInference(const double& transition_probability) :
    robot_state_(RobotState::STANDBY),
    transition_probability_(transition_probability),
    is_inference_activated_(false),
    is_reject_cooldown_timer_started_(false),
    is_assist_dwell_timer_started_(false),
    is_reset_pose_signal_transferred_(false),
    is_reset_pose_completed_(false) {
  belief_ = {};
  position_wrt_odom_ = {};
  position_wrt_tm_base_ = {};
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void IntentInference::setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects) {
  recorded_objects_ = recorded_objects;
  belief_ = {};
  position_wrt_odom_ = {};
  position_wrt_tm_base_ = {};
  for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
    belief_[obj] = 1.0 / recorded_objects.objects.size();
    geometry_msgs::PointStamped centroid;
    centroid.header.frame_id = obj.frame;
    centroid.point = obj.centroid;
    position_wrt_odom_[obj] = tf2_listener_.transformData(centroid, "odom");
    position_wrt_tm_base_[obj] = tf2_listener_.transformData(centroid, "tm_base");
  }
}

 visualization_msgs::MarkerArray IntentInference::getBeliefVisualization()  {
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
    text_marker.scale.z = 0.03;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = std::to_string(belief_value);
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
  gripper_marker.scale.x = 0.1;
  gripper_marker.scale.y = 0.1;
  gripper_marker.scale.z = 0.1;
  gripper_marker.color.a = 0.2;
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
    if (this->isNearTarget())
      this->removeTargetFromRecordedObjects();
    robot_state_ = RobotState::PICK_GRASP;
  } else if (this->isNearTarget()) {
    // ----- Priority 3: gripper is near target (PICK_READY) -----
    if (robot_state_ == RobotState::PICK_ASSIST && !this->isTargetReached()) {
      // [Exception] remain PICK_ASSIST if the planned position has not been reached
      robot_state_ = RobotState::PICK_ASSIST;
    } else {
      /// TODO: Make sure this function works fine
      if (is_reset_pose_signal_transferred_ == false) {
        if (reset_pose_handler_) {
          is_reset_pose_completed_ = reset_pose_handler_();
          is_reset_pose_signal_transferred_ = true;
        } else {
          ROS_WARN("reset_pose_handler_");
        }
      }

      if (is_reset_pose_completed_) {
        robot_state_ = RobotState::PICK_READY;
      }
    }
  } else {
    // ----- Finite State Machine -----
    switch (robot_state_) {
      case RobotState::STANDBY:
        if (is_reject_cooldown_timer_started_) {
          // The PICK_REJECT cooldown has not finished
          // -> back to PICK_REJECT
          robot_state_ = RobotState::PICK_REJECT;
        } else {
          robot_state_ = RobotState::PICK_MANUAL;
        }
      break;
      case RobotState::PICK_MANUAL:
        if (this->isInPickArea() && this->isTowardTarget() && this->isConfidenceHighEnough()) {
          if (!is_assist_dwell_timer_started_) {
            this->triggerAssistDwellTimer();
            is_assist_dwell_timer_started_ = true;
          }

          if (this->isAssistDwellTimePassed()) {
            robot_state_ = RobotState::PICK_ASSIST;
            is_assist_dwell_timer_started_ = false;
          }
        } else {
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
        // else: remain LOCKED
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
        // else: remain RETREATED
      break;
      case RobotState::PICK_READY:
        // Not near the target anymore -> transfer back to PICK_MANUAL
        robot_state_ = RobotState::PICK_MANUAL;
      break;
      default: break;
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
    const double direction_likelihood = (direction_similarity + 1) / 2;  // [-1, 1] -> [0, 1]

    // (2) proximity likelihood
    const double goal_distance = std::sqrt(
        goal_direction.x * goal_direction.x +
        goal_direction.y * goal_direction.y +
        goal_direction.z * goal_direction.z
    );
    const double proximity_likelihood = std::exp(-kProximityLikelihoodParameter * goal_distance);

    // Markov transition update
    const double likelihood = direction_likelihood * proximity_likelihood;
    double sum_transitions = 0.0;
    for (const auto& [object, probability] : belief_) {
      const double transition = (recorded_object == object)
          ? 1.0 - transition_probability_
          : transition_probability_ / (belief_.size() - 1);
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
              return obj.label == target.label && obj.frame == target.frame && obj.centroid == target.centroid;
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

const bool IntentInference::isNearTarget() const {
  const geometry_msgs::Vector3 target_direction = this->getTargetDirection();
  if (target_direction == geometry_msgs::Vector3())  return false;
  const double target_distance = std::sqrt(
      target_direction.x * target_direction.x +
      target_direction.y * target_direction.y +
      target_direction.z * target_direction.z
  );
  return target_distance < kNearDistanceThreshold;
}

const bool IntentInference::isTargetReached() const {
  const geometry_msgs::Point target_position = this->getTargetPosition();
  const double distance = std::sqrt(
      (target_position.x - gripper_position_.x) * (target_position.x - gripper_position_.x) +
      (target_position.y - gripper_position_.y) * (target_position.y - gripper_position_.y) +
      (target_position.z - gripper_position_.z) * (target_position.z - gripper_position_.z)
  );
  return distance < kPositionTolerance;
}

const bool IntentInference::isInPickArea() const {
  for (const auto& [object, prob] : belief_) {
    const double dx = position_wrt_tm_base_.at(object).point.x - gripper_position_.x;
    const double dy = position_wrt_tm_base_.at(object).point.y - gripper_position_.y;
    const double dz = position_wrt_tm_base_.at(object).point.z - gripper_position_.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < kPickAreaDistanceThreshold) {
      return true;
    }
  }
  return false;
}

const bool IntentInference::isTowardTarget() const {
  const geometry_msgs::Vector3 target_direction = this->getTargetDirection();
  const double mag = std::sqrt(
      target_direction.x * target_direction.x +
      target_direction.y * target_direction.y +
      target_direction.z * target_direction.z
  );
  if (mag < 1e-6)  return false;  // avoid divide-by-zero
  
  const double direction_similarity = this->getCosineSimilarity(
      user_command_velocity_,
      target_direction
  );
  return direction_similarity > kTowardTargetDirectionSimilarityThreshold;
}

void IntentInference::triggerAssistDwellTimer() {
  assist_dwell_timer_.start_time = assist_dwell_timer_.current_time = ros::Time::now();
}

const bool IntentInference::isAssistDwellTimePassed() {
  assist_dwell_timer_.current_time = ros::Time::now();
  return assist_dwell_timer_.getTimeDifference() >= kAssistDwellTime;
}

const bool IntentInference::isAwayFromTarget() const {
  const geometry_msgs::Vector3 target_direction = this->getTargetDirection();
  const double mag = std::sqrt(
      target_direction.x * target_direction.x +
      target_direction.y * target_direction.y +
      target_direction.z * target_direction.z
  );
  if (mag < 1e-6)  return false;  // avoid divide-by-zero

  const double direction_similarity = this->getCosineSimilarity(
      user_command_velocity_,
      target_direction
  );
  return direction_similarity < kAwayTargetDirectionSimilarityThreshold;
}

void IntentInference::triggerRejectCooldownTimer() {
  cooldown_timer_.start_time = cooldown_timer_.current_time = ros::Time::now();
}

const bool IntentInference::isRejectCooldownTimePassed() {
  cooldown_timer_.current_time = ros::Time::now();
  return cooldown_timer_.getTimeDifference() >= kRejectCooldownTime;
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
  std::vector<double> probs;
  for (const auto& [_, prob] : belief_) probs.push_back(prob);
  std::sort(probs.begin(), probs.end(), std::greater<double>());

  if (probs.size() >= 2) {
    // confidence -> highest probability - second highest probability
    confidence_ = probs[0] - probs[1];
  } else if (probs.size() == 1) {
    // only one object -> full confidence
    confidence_ = 1.0;
  } else {
    // no objects -> no confidence
    confidence_ = 0.0;
  }
}