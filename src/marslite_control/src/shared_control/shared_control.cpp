#include "shared_control/shared_control.h"

SharedControl::SharedControl()
    : transition_prob_(0.1) {
  // Initialize goals with some example positions
  goals_["goal1"] = {1.0, 0.0, 0.0};
  goals_["goal2"] = {0.0, 1.0, 0.0};
  goals_["goal3"] = {0.0, 0.0, 1.0};

  double uniform = 1.0 / goals_.size();
  for (const auto& [name, _] : goals_) {
    belief_[name] = uniform;
  }
}