#ifndef MARSLITE_CONTROl_SHARED_CONTROL_CONTROLLER_DIRECTION_ESTIMATOR_H
#define MARSLITE_CONTROl_SHARED_CONTROL_CONTROLLER_DIRECTION_ESTIMATOR_H

#include <deque>
#include <geometry_msgs/Point.h>

class ControllerDirectionEstimator {
public:
  ControllerDirectionEstimator(const size_t& max_size = 20);
  void addwaypoint(const geometry_msgs::Point& point);
  inline size_t getSize() const { return waypoint_buffer_.size(); }
  geometry_msgs::Point getAveragedDirection() const;

private:
  std::deque<geometry_msgs::Point> waypoint_buffer_;
  size_t max_size_;

private:
  inline bool isStopped() const {
    if (waypoint_buffer_.size() < 3) return true;
    const geometry_msgs::Point& last_1 = waypoint_buffer_.back();
    const geometry_msgs::Point& last_2 = waypoint_buffer_[waypoint_buffer_.size() - 2];
    const geometry_msgs::Point& last_3 = waypoint_buffer_[waypoint_buffer_.size() - 3];
    return isSameNumber(last_1.x, last_2.x) && isSameNumber(last_1.x, last_3.x)
        && isSameNumber(last_1.y, last_2.y) && isSameNumber(last_1.y, last_3.y)
        && isSameNumber(last_1.z, last_2.z) && isSameNumber(last_1.z, last_3.z);
  }

  inline bool isSameNumber(const double& a, const double& b) const {
    return std::abs(a - b) < 1e-6;
  }
};

#endif // #ifndef MARSLITE_CONTROl_SHARED_CONTROL_CONTROLLER_DIRECTION_ESTIMATOR_H