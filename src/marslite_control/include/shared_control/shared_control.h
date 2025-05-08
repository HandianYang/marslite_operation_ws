#ifndef MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H
#define MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

class SharedControl {
public:
  explicit SharedControl();

  void setRobotPosition(const geometry_msgs::Point& position);
  void setGoals(const std::map<std::string, geometry_msgs::Point>& goals);
  bool updateBelief(const geometry_msgs::Point& joystick_dir);
  std::string getMostLikelyGoal() const;
  std::map<std::string, double> getBeliefs() const;

private:
  std::map<std::string, geometry_msgs::Point> goals_;
  std::map<std::string, double> belief_;
  geometry_msgs::Point robot_position_;
  double transition_prob_;

  double cosineSimilarity3D(const geometry_msgs::Point& a, const geometry_msgs::Point& b) const;
  void normalizeBelief();
};

#endif //MARSLITE_CONTROL_SHARED_CONTROL_SHARED_CONTROL_H