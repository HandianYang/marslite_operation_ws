#ifndef MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
#define MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include "detection_msgs/DetectedObject.h"
#include "detection_msgs/DetectedObjectArray.h"

#include <map>

namespace detection_msgs{

struct DetectedObjectComparator {
  inline const bool operator()(
      const detection_msgs::DetectedObject& a,
      const detection_msgs::DetectedObject& b) const {
    return a.centroid.x < b.centroid.x;
  }
};

using DetectedObjectMap = std::map<detection_msgs::DetectedObject, double, DetectedObjectComparator>;

} // namespace detection_msgs


class IntentInference {
public:
  IntentInference(const double& transition_probability = 0.1);
  IntentInference(const detection_msgs::DetectedObjectArray& recorded_objects,
                  const double& transition_probability = 0.1);

  inline void setRecordedObjects(const detection_msgs::DetectedObjectArray& recorded_objects) {
    recorded_objects_ = recorded_objects;
    for (const detection_msgs::DetectedObject& obj : recorded_objects.objects) {
      belief_[obj] = 1.0 / recorded_objects.objects.size();
    }
  }

  inline void setGripperPosition(const geometry_msgs::Point& position) {
    gripper_position_ = position;
  }
  
  inline void setGripperDirection(const geometry_msgs::Point& direction) {
    gripper_direction_ = direction;
  }

  inline detection_msgs::DetectedObject getMostLikelyGoal() const {
    const auto max_it = std::max_element(belief_.begin(), belief_.end(),
                                        [](const auto& a, const auto& b) {
                                            return a.second < b.second;
                                        });
    return max_it->first;
  }

  inline detection_msgs::DetectedObjectMap getBelief() const {
    return belief_;
  }

  bool updateBelief();
  visualization_msgs::MarkerArray getBeliefVisualization() const;


private:
  double transition_probability_;
  geometry_msgs::Point gripper_position_;
  geometry_msgs::Point gripper_direction_;
  detection_msgs::DetectedObjectArray recorded_objects_;
  detection_msgs::DetectedObjectMap belief_;
  
  double getCosineSimilarity(const geometry_msgs::Point& user_direction, const geometry_msgs::Point& goal_direction) const;
  void normalizeBelief();
};

#endif // MARSLITE_CONTROl_SHARED_CONTROL_INTENT_INFERENCE_H
