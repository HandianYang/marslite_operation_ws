#include "shared_control/intent_inference.h"
#include <cmath>
#include <algorithm>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

IntentInference::IntentInference(const double& transition_probability)
    : transition_probability_(transition_probability) {
  belief_ = {};
}

IntentInference::IntentInference(
    const detection_msgs::DetectedObjectArray& recorded_objects,
    const double& transition_probability)
    : transition_probability_(transition_probability) {
  for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
    belief_[obj] = 1.0 / recorded_objects.objects.size();
  }
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

bool IntentInference::updateBelief() {
  if (recorded_objects_.objects.empty()) {
    return false;
  }
  
  const double mag = std::sqrt(gripper_direction_.x * gripper_direction_.x +
                               gripper_direction_.y * gripper_direction_.y +
                               gripper_direction_.z * gripper_direction_.z);
  if (mag < 1e-3) {
    // [GripperMotionState::IDLE]
    return false;
  }

  detection_msgs::DetectedObjectMap new_belief;
  bool has_valid_likelihood = false;

  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    geometry_msgs::Point goal_direction;
    goal_direction.x = recorded_object.centroid.x - gripper_position_.x;
    goal_direction.y = recorded_object.centroid.y - gripper_position_.y;
    goal_direction.z = recorded_object.centroid.z - gripper_position_.z;

    const double similarity = getCosineSimilarity(gripper_direction_, goal_direction);
    if (similarity < 0.5) {
      // [cosine similarity threshold: 60° (cos(60°) = 0.5)]
      // `similarity < 0.5` means the angle between gripper direction and goal
      //   direction is larger than 60°
      new_belief[recorded_object] = 0.0;
      continue;
    }

    // Markov transition update
    const double likelihood = similarity * 2 - 1;  // remap from [0.5, 1] → [0, 1]
    double sum_transitions = 0.0;
    for (const auto& [object, probability] : belief_) {
      const double transition = (recorded_object == object)
          ? 1.0 - transition_probability_
          : transition_probability_ / (belief_.size() - 1);
      sum_transitions += transition * probability;
    }
    new_belief[recorded_object] = likelihood * sum_transitions;
    has_valid_likelihood = true;
  }

  if (!has_valid_likelihood) {
    // [GripperMotionState::RETREATING]
    //   Gripper direction doesn't align with any goal (angle > 60° for all objects)
    return false;
  }

  belief_ = new_belief;
  normalizeBelief();
  return true;
}


visualization_msgs::MarkerArray IntentInference::getBeliefVisualization() const {
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
    marker.pose.position = object.centroid;
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
    text_marker.pose.position = object.centroid;
    text_marker.pose.position.z += 0.15;
    text_marker.scale.z = 0.03;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = std::to_string(belief_value);

    marker_array.markers.push_back(text_marker);
  }
  return marker_array;
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

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
