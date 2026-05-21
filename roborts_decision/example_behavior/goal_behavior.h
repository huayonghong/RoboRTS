/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_GOAL_BEHAVIOR_H
#define ROBORTS_DECISION_GOAL_BEHAVIOR_H

#include "io/io.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../executor/chassis_executor.h"

namespace roborts_decision {

class GoalBehavior {
 public:
  GoalBehavior(ChassisExecutor *&chassis_executor, Blackboard *&blackboard)
      : chassis_executor_(chassis_executor), blackboard_(blackboard) {}

  void Run() {
    if (blackboard_->IsNewGoal()) {
      chassis_executor_->Execute(blackboard_->GetGoal());
    }
  }

  void Cancel() { chassis_executor_->Cancel(); }

  BehaviorState Update() { return chassis_executor_->Update(); }

  ~GoalBehavior() = default;

 private:
  ChassisExecutor *const chassis_executor_;
  Blackboard *const blackboard_;

  geometry_msgs::msg::PoseStamped planning_goal_;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_GOAL_BEHAVIOR_H
