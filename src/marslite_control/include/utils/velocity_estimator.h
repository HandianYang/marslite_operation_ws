#ifndef MARSLTIE_CONTROL_UTILS_DIRECTION_ESTIMATOR_H
#define MARSLTIE_CONTROL_UTILS_DIRECTION_ESTIMATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <deque>

class VelocityEstimator {
 public:
  VelocityEstimator() = default;
  
  inline void setBufferSize(const size_t& size) {
    params.buffer_size = std::max<size_t>(3, size);
  }

  inline void setMinVelocity(const double& min_velocity) {
    params.min_velocity = min_velocity;
  }
  
  inline void clear() {
    waypoint_buffer_.clear();
    estimated_direction_ = estimated_velocity_ = geometry_msgs::Vector3();
  }

  inline void addWaypoint(const geometry_msgs::PointStamped& position) {
    waypoint_buffer_.push_back(position);
    while (waypoint_buffer_.size() > params.buffer_size)
      waypoint_buffer_.pop_front();
  }

  inline geometry_msgs::Vector3 getEstimatedVelocity() const {
    return estimated_velocity_;
  }

  inline geometry_msgs::Vector3 getEstimatedDirection() const {
    return estimated_direction_;
  }
  
  void estimateVelocity();

 private:
  struct Params {
    size_t buffer_size;
    double min_velocity;
  } params;

  geometry_msgs::Vector3 getVelocity() const;
  double getMeanTime() const;
  double getTimeVariance(const double& mean_time) const;
  geometry_msgs::Point getMeanPosition() const;

  static inline double norm(const geometry_msgs::Vector3& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  }

  std::deque<geometry_msgs::PointStamped> waypoint_buffer_;
  geometry_msgs::Vector3 estimated_velocity_;
  geometry_msgs::Vector3 estimated_direction_;
};


#endif // #ifndef MARSLTIE_CONTROL_UTILS_DIRECTION_ESTIMATOR_H