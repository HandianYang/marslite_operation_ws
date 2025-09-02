#include "shared_control/intent_inference.h"
#include <cmath>
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

IntentInference::IntentInference(const double& transition_probability)
    : transition_probability_(transition_probability),
      gripper_motion_state_(GripperMotionState::IDLE) {
  belief_ = {};
  position_to_base_link_ = {};
}

IntentInference::IntentInference(
    const detection_msgs::DetectedObjectArray& recorded_objects,
    const double& transition_probability)
    : transition_probability_(transition_probability),
      gripper_motion_state_(GripperMotionState::IDLE) {
  for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
    belief_[obj] = 1.0 / recorded_objects.objects.size();
    position_to_base_link_[obj] = this->transformOdomToBaseLink(obj.centroid);
  }
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

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
      gripper_marker.color.r = 0.6;
      gripper_marker.color.g = 1.0;
      gripper_marker.color.b = 0.6;
    break;
    case GripperMotionState::RETREATING:
      gripper_marker.color.r = 1.0;
      gripper_marker.color.g = 0.0;
      gripper_marker.color.b = 0.0;
    break;
    case GripperMotionState::IDLE:
    default:
      gripper_marker.color.r = 0.0;
      gripper_marker.color.g = 0.4;
      gripper_marker.color.b = 0.0;
    break;
  }
  marker_array.markers.push_back(gripper_marker);

  return marker_array;
}

void IntentInference::updatePositionToBaseLink() {
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    position_to_base_link_[recorded_object] = this->transformOdomToBaseLink(recorded_object.centroid);
  }
}

bool IntentInference::updateBelief() {
  if (recorded_objects_.objects.empty()) {
    gripper_motion_state_ = GripperMotionState::IDLE;
    return false;
  }
  
  const double mag = std::sqrt(
      gripper_direction_.x * gripper_direction_.x +
      gripper_direction_.y * gripper_direction_.y +
      gripper_direction_.z * gripper_direction_.z
  );
  if (mag < 1e-3) {
    gripper_motion_state_ = GripperMotionState::IDLE;
    return false;
  }

  detection_msgs::ObjectBeliefMap new_belief;
  bool has_valid_likelihood = false;

  const double kappa = 3.0;  // proximity likelihood parameter
  for (const detection_msgs::DetectedObject& recorded_object : recorded_objects_.objects) {
    geometry_msgs::Point goal_direction;
    goal_direction.x = position_to_base_link_[recorded_object].x - gripper_position_.x;
    goal_direction.y = position_to_base_link_[recorded_object].y - gripper_position_.y;
    goal_direction.z = position_to_base_link_[recorded_object].z - gripper_position_.z;

    const double direction_similarity = getCosineSimilarity(gripper_direction_, goal_direction);
    if (direction_similarity < 0) {
      // [cosine similarity threshold: 90° (cos(90°) = 0)]
      // `similarity < 0` means the angle between gripper direction and goal
      //   direction is larger than 90°
      new_belief[recorded_object] = 0.0;
      continue;
    }
    const double direction_likelihood = direction_similarity;  // use raw cosine similarity [0, 1]

    const double distance = std::sqrt(
        goal_direction.x * goal_direction.x +
        goal_direction.y * goal_direction.y +
        goal_direction.z * goal_direction.z
    );
    const double proximity_likelihood = std::exp(-kappa * distance);

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
    has_valid_likelihood = true;
  }

  if (!has_valid_likelihood) {
    // [GripperMotionState::RETREATING]
    //   Gripper direction doesn't align with any goal (angle > 90° for all objects)
    gripper_motion_state_ = GripperMotionState::RETREATING;
    for (auto& [_, prob] : belief_) {
      prob = 1.0 / belief_.size();
    }
    return false;
  }

  belief_ = new_belief;
  normalizeBelief();
  gripper_motion_state_ = GripperMotionState::APPROACHING;
  return true;
}


/******************************************************
 *                  Private members                   *  
 ****************************************************** */

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
