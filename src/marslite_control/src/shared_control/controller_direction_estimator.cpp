#include "shared_control/controller_direction_estimator.h"

ControllerDirectionEstimator::ControllerDirectionEstimator(const size_t& max_size)
    : max_size_(max_size) {}

void ControllerDirectionEstimator::addwaypoint(const geometry_msgs::Point& point) {
  waypoint_buffer_.push_back(point);
  if (waypoint_buffer_.size() > max_size_) {
    waypoint_buffer_.pop_front();
  }
}

geometry_msgs::Point ControllerDirectionEstimator::getAveragedDirection() const {
  if (waypoint_buffer_.size() < 2 || this->isStopped()) {
    return geometry_msgs::Point();
  }

  geometry_msgs::Point direction;
  for (size_t i = 1; i < waypoint_buffer_.size(); ++i) {
    direction.x += waypoint_buffer_[i].x - waypoint_buffer_[i-1].x;
    direction.y += waypoint_buffer_[i].y - waypoint_buffer_[i-1].y;
    direction.z += waypoint_buffer_[i].z - waypoint_buffer_[i-1].z;
  }

  const double mag = std::sqrt(direction.x * direction.x
      + direction.y * direction.y
      + direction.z * direction.z
  );
  if (mag < 1e-3) {
    direction.x = direction.y = direction.z = 0.0;
  } else {
    direction.x /= mag;
    direction.y /= mag;
    direction.z /= mag;
  }
  
  return direction;
}
