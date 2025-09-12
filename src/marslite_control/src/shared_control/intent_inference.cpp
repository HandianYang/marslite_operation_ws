#include "shared_control/intent_inference.h"
#include <cmath>
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

IntentInference::IntentInference(const double& transition_probability,
                                 const double& confidence_lower_bound,
                                 const double& confidence_upper_bound,
                                 const double& alpha_maximum)
    : gripper_motion_state_(GripperMotionState::IDLE),
      confidence_(0.0), alpha_(0.0),
      transition_probability_(transition_probability),
      confidence_lower_bound_(confidence_lower_bound),
      confidence_upper_bound_(confidence_upper_bound),
      alpha_maximum_(alpha_maximum) {
  belief_ = {};
  position_to_base_link_ = {};
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void IntentInference::setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects) {
  recorded_objects_ = recorded_objects;
  belief_ = {};
  position_to_base_link_ = {};
  for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
    belief_[obj] = 1.0 / recorded_objects.objects.size();
    position_to_base_link_[obj] = this->transformOdomToBaseLink(obj.centroid);
  }
}

 visualization_msgs::MarkerArray IntentInference::getBeliefVisualization()  {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& [object, belief_value] : belief_) {
    // object marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "intent_belief";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = position_to_base_link_[object];
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
    text_marker.pose.position = position_to_base_link_[object];
    text_marker.pose.position.z += 0.15;
    text_marker.scale.z = 0.03;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = std::to_string(belief_value);
    marker_array.markers.push_back(text_marker);
  }

  // Gripper status marker
  visualization_msgs::Marker gripper_marker;
  gripper_marker.header.frame_id = "base_link";
  gripper_marker.header.stamp = ros::Time::now();
  gripper_marker.ns = "gripper_status";
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
    case GripperMotionState::APPROACHING:
      // light green (alpha = 0) to dark green (alpha = 1)
      if (confidence_ < 0.3) {
        gripper_marker.color.r = 0.5;
        gripper_marker.color.g = 1.0;
        gripper_marker.color.b = 0.5;
      } else {
        gripper_marker.color.r = 0.0;
        gripper_marker.color.g = 1.0;
        gripper_marker.color.b = 0.0;
      }
    break;
    case GripperMotionState::RETREATING:
      // red
      gripper_marker.color.r = 1.0;
      gripper_marker.color.g = 0.0;
      gripper_marker.color.b = 0.0;
    break;
    case GripperMotionState::NEAR_GOAL:
      // yellow
      gripper_marker.color.r = 1.0;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.0;
    break;
    case GripperMotionState::GRASPING:
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

  // Alpha value text marker
  visualization_msgs::Marker alpha_text_marker;
  alpha_text_marker.header.frame_id = "base_link";
  alpha_text_marker.header.stamp = ros::Time::now();
  alpha_text_marker.ns = "alpha_value_text";
  alpha_text_marker.id = id++;
  alpha_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  alpha_text_marker.action = visualization_msgs::Marker::ADD;
  alpha_text_marker.pose.position.x = gripper_position_.x;
  alpha_text_marker.pose.position.y = gripper_position_.y;
  alpha_text_marker.pose.position.z = gripper_position_.z + 0.2;
  alpha_text_marker.scale.z = 0.04;
  alpha_text_marker.color.r = 1.0;
  alpha_text_marker.color.g = 1.0;
  alpha_text_marker.color.b = 1.0;
  alpha_text_marker.color.a = 1.0;
  alpha_text_marker.text = std::to_string(alpha_);
  marker_array.markers.push_back(alpha_text_marker);

  return marker_array;
}

geometry_msgs::Point IntentInference::getGoalDirection(const detection_msgs::DetectedObject& goal) const { 
  geometry_msgs::Point goal_direction;
  goal_direction.x = position_to_base_link_.at(goal).x - gripper_position_.x;
  goal_direction.y = position_to_base_link_.at(goal).y - gripper_position_.y;
  goal_direction.z = position_to_base_link_.at(goal).z - gripper_position_.z;
  return goal_direction;
}

void IntentInference::updateBelief() {
  this->updatePositionToBaseLink();
  this->updateGripperMotionState();

  if (gripper_motion_state_ == GripperMotionState::IDLE
      || gripper_motion_state_ == GripperMotionState::GRASPING
      || gripper_motion_state_ == GripperMotionState::NEAR_GOAL) {
    alpha_ = 0.0;
    return;
  }

  if (gripper_motion_state_ == GripperMotionState::RETREATING) {
    // -> Gripper direction doesn't align with any goal (angle > 60° for all objects)
    for (auto& [_, prob] : belief_) {
      prob = 1.0 / belief_.size();
    }
    alpha_ = 0.0;
    return;
  }

  // GripperMotionState::APPROACHING
  // -> Update belief using Bayesian inference with Markov transition model
  detection_msgs::ObjectBeliefMap new_belief;
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    geometry_msgs::Point goal_direction = this->getGoalDirection(recorded_object);

    // (1) direction_likelihood
    const double direction_similarity = getCosineSimilarity(gripper_direction_, goal_direction);
    const double direction_likelihood = (direction_similarity + 1) / 2;  // [-1, 1] -> [0, 1]

    // (2) proximity likelihood
    const double distance = std::sqrt(
        goal_direction.x * goal_direction.x +
        goal_direction.y * goal_direction.y +
        goal_direction.z * goal_direction.z
    );
    const double proximity_likelihood = std::exp(-kKappa * distance);

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
  this->calculateAlpha();
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void IntentInference::updatePositionToBaseLink() {
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    position_to_base_link_[recorded_object] =
        this->transformOdomToBaseLink(recorded_object.centroid);
  }
}

geometry_msgs::Point IntentInference::transformOdomToBaseLink(const geometry_msgs::Point& point_in_odom) {
  geometry_msgs::PointStamped point_stamped_in_odom;
  point_stamped_in_odom.header.frame_id = "odom";
  point_stamped_in_odom.header.stamp = ros::Time(0);
  point_stamped_in_odom.point = point_in_odom;
  
  geometry_msgs::TransformStamped odom_to_base_link_transform
      = tf2_listener_.lookupTransform("base_link", "odom");
  geometry_msgs::PointStamped point_stamped_in_base_link;
  tf2::doTransform(point_stamped_in_odom, point_stamped_in_base_link, odom_to_base_link_transform);
  
  return point_stamped_in_base_link.point;
}

void IntentInference::updateGripperMotionState() {
  const double gripper_displacement = std::sqrt(
      gripper_direction_.x * gripper_direction_.x +
      gripper_direction_.y * gripper_direction_.y +
      gripper_direction_.z * gripper_direction_.z
  );

  if (gripper_status_.data) {
    gripper_motion_state_ = GripperMotionState::GRASPING;
  } else if (recorded_objects_.objects.empty() || gripper_displacement < 1e-3) {
    gripper_motion_state_ = GripperMotionState::IDLE;
  } else if (this->isNearAnyGoal()) {
    gripper_motion_state_ = GripperMotionState::NEAR_GOAL;
  }  else if (this->isTowardAnyGoal()) {
    gripper_motion_state_ = GripperMotionState::APPROACHING;
  } else {
    gripper_motion_state_ = GripperMotionState::RETREATING;
  }
}

bool IntentInference::isNearAnyGoal() const {
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    const geometry_msgs::Point goal_direction = this->getGoalDirection(recorded_object);
    const double distance = std::sqrt(
        goal_direction.x * goal_direction.x +
        goal_direction.y * goal_direction.y +
        goal_direction.z * goal_direction.z
    );
    if (distance < 0.03) {
      // [distance threshold: 10cm]
      return true;
    }
  }
  return false;
}

bool IntentInference::isTowardAnyGoal() const {
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    const geometry_msgs::Point goal_direction = this->getGoalDirection(recorded_object);
    const double direction_similarity = getCosineSimilarity(gripper_direction_, goal_direction);
    if (direction_similarity > 0.5) {
      // [cosine similarity threshold: 60° (cos(60°) = 0.5)]
      // `similarity > 0.5` means the angle between gripper direction and goal
      //   direction is smaller than 60°
      return true;
    }
  }
  return false;
}

double IntentInference::getCosineSimilarity(
    const geometry_msgs::Point& user_direction,
    const geometry_msgs::Point& goal_direction) const {
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

void IntentInference::calculateAlpha() {
  if (confidence_ <= confidence_lower_bound_) {
    alpha_ = 0.0;
  } else if (confidence_ <= confidence_upper_bound_) {
    // linear function from 0 to alpha_maximum_
    alpha_ = alpha_maximum_ * (confidence_ - confidence_lower_bound_)
        / (confidence_upper_bound_ - confidence_lower_bound_);
  } else {
    alpha_ = alpha_maximum_;
  }
}