#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include "io/io.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../executor/chassis_executor.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class ChaseBehavior {
 public:
  ChaseBehavior(ChassisExecutor *&chassis_executor, Blackboard *&blackboard,
                const std::string &proto_file_path)
      : chassis_executor_(chassis_executor), blackboard_(blackboard) {
    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    chase_buffer_.resize(2);
    chase_count_ = 0;

    cancel_goal_ = true;
    (void)proto_file_path;
  }

  void Run() {
    auto executor_state = Update();

    auto robot_map_pose = blackboard_->GetRobotMapPose();
    if (executor_state != BehaviorState::RUNNING) {
      chase_buffer_[chase_count_++ % 2] = blackboard_->GetEnemy();

      chase_count_ = chase_count_ % 2;

      auto dx = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose.pose.position.x;
      auto dy = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose.pose.position.y;
      auto yaw = std::atan2(dy, dx);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 &&
          std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
        if (cancel_goal_) {
          chassis_executor_->Cancel();
          cancel_goal_ = false;
        }
        return;
      }

      geometry_msgs::msg::PoseStamped reduce_goal;
      reduce_goal.pose.orientation = robot_map_pose.pose.orientation;

      reduce_goal.header.frame_id = "map";
      reduce_goal.header.stamp = blackboard_->Now();
      reduce_goal.pose.position.x =
          chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - 1.2 * std::cos(yaw);
      reduce_goal.pose.position.y =
          chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - 1.2 * std::sin(yaw);
      auto enemy_x = reduce_goal.pose.position.x;
      auto enemy_y = reduce_goal.pose.position.y;
      reduce_goal.pose.position.z = 1;
      unsigned int goal_cell_x, goal_cell_y;

      auto get_enemy_cell =
          blackboard_->GetCostMap2D()->World2Map(enemy_x, enemy_y, goal_cell_x, goal_cell_y);

      if (!get_enemy_cell) {
        return;
      }

      auto robot_x = robot_map_pose.pose.position.x;
      auto robot_y = robot_map_pose.pose.position.y;
      unsigned int robot_cell_x, robot_cell_y;
      double goal_x = 0, goal_y = 0;
      blackboard_->GetCostMap2D()->World2Map(robot_x, robot_y, robot_cell_x, robot_cell_y);

      if (blackboard_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253) {
        bool find_goal = false;
        for (FastLineIterator line(goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid();
             line.Advance()) {
          auto point_cost = blackboard_->GetCostMap2D()->GetCost(static_cast<unsigned int>(line.GetX()),
                                                                  static_cast<unsigned int>(line.GetY()));

          if (point_cost >= 253) {
            continue;
          }
          find_goal = true;
          blackboard_->GetCostMap2D()->Map2World(static_cast<unsigned int>(line.GetX()),
                                                 static_cast<unsigned int>(line.GetY()), goal_x, goal_y);

          reduce_goal.pose.position.x = goal_x;
          reduce_goal.pose.position.y = goal_y;
          break;
        }
        if (find_goal) {
          cancel_goal_ = true;
          chassis_executor_->Execute(reduce_goal);
        } else {
          if (cancel_goal_) {
            chassis_executor_->Cancel();
            cancel_goal_ = false;
          }
        }
        return;
      }

      cancel_goal_ = true;
      chassis_executor_->Execute(reduce_goal);
    }
  }

  void Cancel() { chassis_executor_->Cancel(); }

  BehaviorState Update() { return chassis_executor_->Update(); }

  void SetGoal(const geometry_msgs::msg::PoseStamped &chase_goal) { chase_goal_ = chase_goal; }

  ~ChaseBehavior() = default;

 private:
  ChassisExecutor *const chassis_executor_;
  Blackboard *const blackboard_;
  geometry_msgs::msg::PoseStamped chase_goal_;
  std::vector<geometry_msgs::msg::PoseStamped> chase_buffer_;
  unsigned int chase_count_;
  bool cancel_goal_;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_CHASE_BEHAVIOR_H
