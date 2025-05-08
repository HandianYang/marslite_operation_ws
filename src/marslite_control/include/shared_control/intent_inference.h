#ifndef MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
#define MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <map>
#include <string>
#include <vector>

class IntentInference {
public:
    IntentInference(const std::map<std::string, geometry_msgs::Point>& goal_positions,
                    double transition_prob = 0.1);

    void setRobotPosition(const geometry_msgs::Point& position);
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

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
