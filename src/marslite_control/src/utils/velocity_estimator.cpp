#include "utils/velocity_estimator.h"

void VelocityEstimator::estimateVelocity() {
  estimated_velocity_ = geometry_msgs::Vector3();
  estimated_direction_ = geometry_msgs::Vector3();
  
  if (waypoint_buffer_.size() < 3)
    return;
  
  const geometry_msgs::Vector3 raw_velocity = this->getVelocity();
  const double raw_speed = norm(raw_velocity);
  if (raw_speed < params.min_speed) {
    estimated_velocity_ = geometry_msgs::Vector3();
    estimated_direction_ = geometry_msgs::Vector3();
    previous_filtered_velocity_ = geometry_msgs::Vector3();
    return;
  }

  if (previous_filtered_velocity_.x == 0 && previous_filtered_velocity_.y == 0)
    previous_filtered_velocity_ = raw_velocity;

  geometry_msgs::Vector3 smooth_velocity;
  const double alpha = 0.3; // the smaller the smoother (but more delay)
  smooth_velocity.x = alpha * raw_velocity.x + (1.0 - alpha) * previous_filtered_velocity_.x;
  smooth_velocity.y = alpha * raw_velocity.y + (1.0 - alpha) * previous_filtered_velocity_.y;
  smooth_velocity.z = alpha * raw_velocity.z + (1.0 - alpha) * previous_filtered_velocity_.z;
  estimated_velocity_ = previous_filtered_velocity_ = smooth_velocity;

  const double smooth_speed = norm(smooth_velocity);
  if (smooth_speed > 1e-6) {
    estimated_direction_.x = smooth_velocity.x / smooth_speed;
    estimated_direction_.y = smooth_velocity.y / smooth_speed;
    estimated_direction_.z = smooth_velocity.z / smooth_speed;
  }
}

geometry_msgs::Vector3 VelocityEstimator::getVelocity() const {
  const double mean_time = this->getMeanTime();
  const double time_variance = this->getTimeVariance(mean_time);
  const geometry_msgs::Point mean_position = this->getMeanPosition();
  if (time_variance < 1e-9) return geometry_msgs::Vector3();

  geometry_msgs::Vector3 velocity_sum;
  for (const geometry_msgs::PointStamped& waypoint : waypoint_buffer_) {
    const double dt = waypoint.header.stamp.toSec() - mean_time;
    velocity_sum.x += dt * (waypoint.point.x - mean_position.x);
    velocity_sum.y += dt * (waypoint.point.y - mean_position.y);
    velocity_sum.z += dt * (waypoint.point.z - mean_position.z);
  }
  geometry_msgs::Vector3 velocity;
  velocity.x = velocity_sum.x / time_variance;
  velocity.y = velocity_sum.y / time_variance;
  velocity.z = velocity_sum.z / time_variance;
  return velocity;
}

double VelocityEstimator::getMeanTime() const {
  double mean_time = 0.0;
  for (const geometry_msgs::PointStamped& waypoint : waypoint_buffer_)
    mean_time += waypoint.header.stamp.toSec();
  mean_time /= static_cast<double>(waypoint_buffer_.size());
  return mean_time;
}

double VelocityEstimator::getTimeVariance(const double& mean_time) const {
  double time_variance = 0.0;
  for (const geometry_msgs::PointStamped& waypoint : waypoint_buffer_) {
    const double dt = waypoint.header.stamp.toSec() - mean_time;
    time_variance += dt * dt;
  }
  return time_variance;
}

geometry_msgs::Point VelocityEstimator::getMeanPosition() const {
  geometry_msgs::Point mean_position;
  for (const geometry_msgs::PointStamped& waypoint : waypoint_buffer_) {
    mean_position.x += waypoint.point.x;
    mean_position.y += waypoint.point.y;
    mean_position.z += waypoint.point.z;
  }
  mean_position.x /= waypoint_buffer_.size();
  mean_position.y /= waypoint_buffer_.size();
  mean_position.z /= waypoint_buffer_.size();
  return mean_position;
}