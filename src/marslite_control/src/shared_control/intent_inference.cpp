#include "shared_control/intent_inference.h"
#include <cmath>
#include <algorithm>

IntentInference::IntentInference(const std::map<std::string, geometry_msgs::Point>& goal_positions,
                                 double transition_prob)
    : goals_(goal_positions), transition_prob_(transition_prob) {
  double uniform = 1.0 / goals_.size();
  for (const auto& [name, _] : goals_) {
    belief_[name] = uniform;
  }
}

void IntentInference::setRobotPosition(const geometry_msgs::Point& position) {
  robot_position_ = position;
}

double IntentInference::cosineSimilarity3D(const geometry_msgs::Point& a,
                                           const geometry_msgs::Point& b) const {
  double dot = a.x * b.x + a.y * b.y + a.z * b.z;
  double mag_a = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  double mag_b = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
  if (mag_a == 0.0 || mag_b == 0.0) return 0.0;
  return dot / (mag_a * mag_b);
}

void IntentInference::normalizeBelief() {
  double sum = 0.0;
  for (const auto& [_, prob] : belief_) sum += prob;
  for (auto& [_, prob] : belief_) prob /= (sum + 1e-9); // avoid divide-by-zero
}

bool IntentInference::updateBelief(const geometry_msgs::Point& joystick_dir) {
  // Ignore update if joystick magnitude is too low (idle)
  double mag = std::sqrt(joystick_dir.x * joystick_dir.x +
                         joystick_dir.y * joystick_dir.y +
                         joystick_dir.z * joystick_dir.z);
  if (mag < 1e-3) {
    ROS_WARN_THROTTLE(2.0, "Joystick input too small. Skipping update.");
    return false;
  }

  std::map<std::string, double> new_belief;
  bool has_valid_likelihood = false;

  for (const auto& [goal_name, goal_pos] : goals_) {
    // 1. Compute direction vector from robot to goal
    geometry_msgs::Point goal_dir;
    goal_dir.x = goal_pos.x - robot_position_.x;
    goal_dir.y = goal_pos.y - robot_position_.y;
    goal_dir.z = goal_pos.z - robot_position_.z;

    // 2. Compute cosine similarity
    double similarity = cosineSimilarity3D(joystick_dir, goal_dir);

    // 3. Threshold: use only if angle ≤ 45° (cos(45°) ≈ 0.707)
    double likelihood = 0.0;
    if (similarity >= 0.707) {
      likelihood = (similarity + 1.0) / 2.0;  // remap from [-1, 1] → [0, 1]
      has_valid_likelihood = true;
    }

    // 4. Markov transition update
    double sum_transitions = 0.0;
    for (const auto& [prev_goal, prob] : belief_) {
      double transition = (prev_goal == goal_name)
          ? 1.0 - transition_prob_
          : transition_prob_ / (belief_.size() - 1);
      sum_transitions += transition * prob;
    }

    new_belief[goal_name] = likelihood * sum_transitions;
  }

  // 5. Recovery: if all likelihoods are 0, skip update
  if (!has_valid_likelihood) {
    ROS_WARN_THROTTLE(2.0, "Joystick direction doesn't align with any goal (angle > 45°). Skipping update.");
    return false;
  }

  // 6. Normalize and update belief
  belief_ = new_belief;
  normalizeBelief();
  return true;
}


std::string IntentInference::getMostLikelyGoal() const {
  auto max_it = std::max_element(belief_.begin(), belief_.end(),
                                [](const auto& a, const auto& b) {
                                    return a.second < b.second;
                                });
  return max_it->first;
}

std::map<std::string, double> IntentInference::getBeliefs() const {
  return belief_;
}
