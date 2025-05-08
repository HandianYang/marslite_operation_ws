#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <map>
#include "shared_control/intent_inference.h"

geometry_msgs::Point joystick_vector;

bool isZeroVector(const geometry_msgs::Point& vec) {
  return std::abs(vec.x) < 1e-9 && std::abs(vec.y) < 1e-9 && std::abs(vec.z) < 1e-9;
}

void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // normalized joystick vector
  joystick_vector.y = msg->axes[0];
  joystick_vector.x = -msg->axes[1];
  joystick_vector.z = 0.0;
  // std::cout << x << " " << y << std::endl;
}

geometry_msgs::Point blendWaypoints(
    const geometry_msgs::Point& p_user,
    const geometry_msgs::Point& p_robot,
    double alpha) {
  geometry_msgs::Point p_blend;
  p_blend.x = p_user.x * (1.0 - alpha) + p_robot.x * alpha;
  p_blend.y = p_user.y * (1.0 - alpha) + p_robot.y * alpha;
  p_blend.z = p_user.z * (1.0 - alpha) + p_robot.z * alpha;
  return p_blend;
}

geometry_msgs::Point computeAutonomyWaypoint(
    const geometry_msgs::Point& ee_pos,
    const geometry_msgs::Point& goal_pos,
    double step_size = 0.01) {
  geometry_msgs::Point delta;
  delta.x = goal_pos.x - ee_pos.x;
  delta.y = goal_pos.y - ee_pos.y;
  delta.z = goal_pos.z - ee_pos.z;

  double mag = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  if (mag > 1e-6) {
    delta.x = delta.x / mag * step_size;
    delta.y = delta.y / mag * step_size;
    delta.z = delta.z / mag * step_size;
  } else {
    delta.x = delta.y = delta.z = 0.0;
  }

  geometry_msgs::Point autonomy_wp;
  autonomy_wp.x = ee_pos.x + delta.x;
  autonomy_wp.y = ee_pos.y + delta.y;
  autonomy_wp.z = ee_pos.z + delta.z;

  return autonomy_wp;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_intent_inference");
  ros::NodeHandle nh;
  ros::Subscriber left_joy_sub = nh.subscribe("/unity/joy/left", 10, leftJoyCallback);

  std::map<std::string, geometry_msgs::Point> goal_map;
  geometry_msgs::Point goal;
  // goal.x = 1.0;   goal.y = 0.0;   goal.z = 0.0;
  // goal_map["goal-east"] = goal;
  // goal.x = -1.0;  goal.y = 0.0;   goal.z = 0.0;
  // goal_map["goal-west"] = goal;
  // goal.x = 0.0;   goal.y = 1.0;   goal.z = 0.0;
  // goal_map["goal-north"] = goal;
  // goal.x = 0.0;   goal.y = -1.0;  goal.z = 0.0;
  // goal_map["goal-south"] = goal;
  goal.x = 3.0;   goal.y = 2.0;   goal.z = 0.0;
  goal_map["A"] = goal;
  goal.x = 3.0;   goal.y = 1.0;   goal.z = 0.0;
  goal_map["B"] = goal;
  goal.x = 3.0;   goal.y = 0.0;   goal.z = 0.0;
  goal_map["C"] = goal;
  goal.x = 3.0;   goal.y = -1.0;  goal.z = 0.0;
  goal_map["D"] = goal;
  goal.x = 3.0;   goal.y = -2.0;  goal.z = 0.0;
  goal_map["E"] = goal;

  IntentInference intent_infer(goal_map);
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  intent_infer.setRobotPosition(robot_position);

  std::map<std::string, double> belief;
  std::string top_goal;
  while (ros::ok())
  {
    if (intent_infer.updateBelief(joystick_vector)) {
      belief = intent_infer.getBeliefs();
      top_goal = intent_infer.getMostLikelyGoal();
      
      std::cout << "Beliefs: ";
      for (const auto& [goal_name, prob] : belief) {
        std::cout << goal_name << ": " << prob << " ";
      }
      std::cout << std::endl;
    }
    
    ros::Rate(25).sleep();
    ros::spinOnce();
  }

  return 0;
}