#include "shared_control/intent_inference.h"
#include <ros/time.h>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

IntentInference::IntentInference(const double& transition_probability)
    : gripper_motion_state_(GripperMotionState::IDLE),
      last_gripper_motion_state_(GripperMotionState::IDLE),
      transition_probability_(transition_probability),
      is_cooldown_timer_started_(false),
      is_locked_timer_started_(false) {
  belief_ = {};
  position_to_tm_base_ = {};
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void IntentInference::setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects) {
  recorded_objects_ = recorded_objects;
  belief_ = {};
  position_to_tm_base_ = {};
  for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
    belief_[obj] = 1.0 / recorded_objects.objects.size();
    position_to_tm_base_[obj] = this->transformOdomToTMBase(obj.centroid);
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
    marker.pose.position = position_to_tm_base_[object];
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0 - belief_value;
    marker.color.g = belief_value;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker_array.markers.push_back(marker);

    // text marker
    visualization_msgs::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "intent_belief_text";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = position_to_tm_base_[object];
    text_marker.pose.position.z += 0.15;
    text_marker.scale.z = 0.03;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = std::to_string(belief_value);
    marker_array.markers.push_back(text_marker);
  }

  // gripper_motion_state marker
  visualization_msgs::Marker gripper_marker;
  gripper_marker.header.frame_id = "tm_base";
  gripper_marker.header.stamp = ros::Time::now();
  gripper_marker.ns = "gripper_motion_state";
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

  switch (gripper_motion_state_) {
    case GripperMotionState::UNLOCKED:
      // light green
      gripper_marker.color.r = 0.6;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.6;
    break;
    case GripperMotionState::LOCKED:
      // green
      gripper_marker.color.r = 0.0;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.0;
    break;
    case GripperMotionState::RETREATED:
      // red
      gripper_marker.color.r = 1.0;
      gripper_marker.color.g = 0.0;
      gripper_marker.color.b = 0.0;
    break;
    case GripperMotionState::REACHED:
      // yellow
      gripper_marker.color.r = 1.0;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.0;
    break;
    case GripperMotionState::GRASPED:
      // blue
      gripper_marker.color.r = 0.0;
      gripper_marker.color.g = 0.0;
      gripper_marker.color.b = 1.0;
    break;
    case GripperMotionState::IDLE:
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

void IntentInference::updateBelief() {
  this->updatePositionToBaseLink();
  this->updateGripperMotionState();
  if (gripper_motion_state_ != GripperMotionState::UNLOCKED)  return;

  // Update belief using Bayesian inference with Markov transition model
  detection_msgs::ObjectBeliefMap new_belief;
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    geometry_msgs::Vector3 goal_direction = this->getGoalDirection(recorded_object);

    // (1) direction_likelihood
    // TODO: Test new likelihood
    const double direction_similarity = getCosineSimilarity(user_command_velocity_, goal_direction);
    const double direction_likelihood = (direction_similarity + 1) / 2;  // [-1, 1] -> [0, 1]
    // const double direction_likelihood = std::exp(direction_similarity - 1); // [-1, 1] -> [e^(-2), e^0]

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

void IntentInference::updatePositionToBaseLink() {
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    position_to_tm_base_[recorded_object] =
        this->transformOdomToTMBase(recorded_object.centroid);
  }
}

geometry_msgs::Point IntentInference::transformOdomToTMBase(const geometry_msgs::Point& point_in_odom) {
  geometry_msgs::PointStamped point_stamped_in_odom;
  point_stamped_in_odom.header.frame_id = "odom";
  point_stamped_in_odom.header.stamp = ros::Time(0);
  point_stamped_in_odom.point = point_in_odom;
  
  geometry_msgs::TransformStamped odom_to_tm_base_transform
      = tf2_listener_.lookupTransform<geometry_msgs::TransformStamped>("tm_base", "odom");
  geometry_msgs::PointStamped point_stamped_in_tm_base;
  tf2::doTransform(point_stamped_in_odom, point_stamped_in_tm_base, odom_to_tm_base_transform);
  
  return point_stamped_in_tm_base.point;
}

void IntentInference::updateGripperMotionState() {
  last_gripper_motion_state_ = gripper_motion_state_;
  if (last_gripper_motion_state_ == GripperMotionState::UNLOCKED || 
      last_gripper_motion_state_ == GripperMotionState::LOCKED ||
      last_gripper_motion_state_ == GripperMotionState::REACHED) {
    this->calculatePlannedPositionAndTargetDirection();
  }

  if (!is_positional_safety_button_pressed_ || recorded_objects_.objects.empty()) {
    gripper_motion_state_ = GripperMotionState::IDLE;
  } else if (gripper_status_.data) {
    gripper_motion_state_ = GripperMotionState::GRASPED;
  } else if (this->isNearTarget()) {
    if (last_gripper_motion_state_ == GripperMotionState::LOCKED && !this->isPlannedPositionReached()) {
      // [Exception] remain LOCKED if the planned position has not been reached
      gripper_motion_state_ = GripperMotionState::LOCKED;
    } else {
      gripper_motion_state_ = GripperMotionState::REACHED;
    }
  } else {
    // finite state machine
    switch (last_gripper_motion_state_) {
      case GripperMotionState::IDLE:
        if (is_cooldown_timer_started_) {
          // The RETREATED cooldown has not finished
          gripper_motion_state_ = GripperMotionState::RETREATED;
        } else {
          gripper_motion_state_ = GripperMotionState::UNLOCKED;
        }
      break;
      case GripperMotionState::UNLOCKED:
        if (this->isTowardTarget() && confidence_ >= kLockTargetConfidenceThreshold) {
          if (!is_locked_timer_started_) {
            this->triggerLockedTimer();
            is_locked_timer_started_ = true;
          }
          if (this->isLockedTimePassed()) {
            gripper_motion_state_ = GripperMotionState::LOCKED;
            is_locked_timer_started_ = false;
          }
        } else {
          // remain UNLOCKED
          is_locked_timer_started_ = false;
        } 
      break;
      case GripperMotionState::LOCKED:
        if (this->isAwayFromTarget()) {
          // ROS_INFO_STREAM("LOCKED to RETREATED. is_cooldown_timer_started_: " << is_cooldown_timer_started_);
          gripper_motion_state_ = GripperMotionState::RETREATED;
        }
        // else: remain LOCKED
      break;
      case GripperMotionState::RETREATED:
        if (!is_cooldown_timer_started_) {
          // ROS_INFO("RETREATED cooldown started");
          this->triggerCooldownTimer();
          is_cooldown_timer_started_ = true;
        }
        if (this->isCooldownTimePassed()) {
          // ROS_INFO("RETREATED to UNLOCKED");
          gripper_motion_state_ = GripperMotionState::UNLOCKED;
          is_cooldown_timer_started_ = false;
        }
        // else: remain RETREATED
      break;
      default: break;
    }
  }
  // ROS_INFO_STREAM("New state: " << toString(gripper_motion_state_));
}

void IntentInference::calculatePlannedPositionAndTargetDirection() {
  // Shift the position ahead kTargetShiftDistance
  geometry_msgs::Point target_position = this->getMostLikelyGoalPosition();
  target_direction_.x = target_position.x - gripper_position_.x;
  target_direction_.y = target_position.y - gripper_position_.y;
  target_direction_.z = target_position.z - gripper_position_.z;
  const double target_distance = std::sqrt(
    target_direction_.x * target_direction_.x + 
    target_direction_.y * target_direction_.y +
    target_direction_.z * target_direction_.z
  );
  geometry_msgs::Vector3 scaled_target_direction;
  scaled_target_direction.x = target_direction_.x / target_distance * kTargetShiftDistance;
  scaled_target_direction.y = target_direction_.y / target_distance * kTargetShiftDistance;
  scaled_target_direction.z = target_direction_.z / target_distance * kTargetShiftDistance;

  planned_position_.x = target_position.x - scaled_target_direction.x;
  planned_position_.y = target_position.y - scaled_target_direction.y;
  planned_position_.z = target_position.z - scaled_target_direction.z;
}

const bool IntentInference::isNearTarget() const {
  if (this->isEmptyTargetDirection())  return false;
  const double target_distance = std::sqrt(
      target_direction_.x * target_direction_.x + 
      target_direction_.y * target_direction_.y +
      target_direction_.z * target_direction_.z
  );
  return target_distance < kNearDistanceThreshold;
}

const bool IntentInference::isPlannedPositionReached() const {
  geometry_msgs::Vector3 direction_to_planned_position;
  direction_to_planned_position.x = planned_position_.x - gripper_position_.x;
  direction_to_planned_position.y = planned_position_.y - gripper_position_.y;
  direction_to_planned_position.z = planned_position_.z - gripper_position_.z;
  const double distance_to_planned_position = std::sqrt(
      direction_to_planned_position.x * direction_to_planned_position.x + 
      direction_to_planned_position.y * direction_to_planned_position.y +
      direction_to_planned_position.z * direction_to_planned_position.z
  );
  return distance_to_planned_position < kPositionTolerance;
}

void IntentInference::triggerLockedTimer() {
  locked_timer_.start_time = locked_timer_.current_time = ros::Time::now();
}

const bool IntentInference::isLockedTimePassed() {
  locked_timer_.current_time = ros::Time::now();
  return locked_timer_.getTimeDifference() >= kLockedTime;
}

const bool IntentInference::isTowardTarget() const {
  if (this->isEmptyTargetDirection())  return false;
  const double direction_similarity = this->getCosineSimilarity(user_command_velocity_, target_direction_);
  return direction_similarity > kLockedSimilarityThreshold;
}

const bool IntentInference::isAwayFromTarget() const {
  if (this->isEmptyTargetDirection())  return false;
  const double direction_similarity = this->getCosineSimilarity(user_command_velocity_, target_direction_);
  return direction_similarity < kRetreatSimilarityThreshold;
}

void IntentInference::triggerCooldownTimer() {
  cooldown_timer_.start_time = cooldown_timer_.current_time = ros::Time::now();
}

const bool IntentInference::isCooldownTimePassed() {
  cooldown_timer_.current_time = ros::Time::now();
  return cooldown_timer_.getTimeDifference() >= kRetreatCooldownTime;
}

geometry_msgs::Vector3 IntentInference::getGoalDirection(const detection_msgs::DetectedObject& goal) const { 
  geometry_msgs::Vector3 goal_direction;
  goal_direction.x = position_to_tm_base_.at(goal).x - gripper_position_.x;
  goal_direction.y = position_to_tm_base_.at(goal).y - gripper_position_.y;
  goal_direction.z = position_to_tm_base_.at(goal).z - gripper_position_.z;
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
    // confidence = highest probability - second highest probability
    confidence_ = probs[0] - probs[1];
  } else if (probs.size() == 1) {
    // only one object -> full confidence
    confidence_ = 1.0;
  } else {
    // no objects -> no confidence
    confidence_ = 0.0;
  }
}