/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_ESCAPEBEHAVIOR_H
#define ROBORTS_DECISION_ESCAPEBEHAVIOR_H

#include <cmath>
#include <random>

#include "io/io.h"

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <roborts_msgs/msg/twist_accel.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../executor/chassis_executor.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"
#include "msg_helpers.hpp"

namespace roborts_decision {

class EscapeBehavior {
 public:
  EscapeBehavior(ChassisExecutor *&chassis_executor, Blackboard *&blackboard,
                 const std::string &proto_file_path)
      : chassis_executor_(chassis_executor), blackboard_(blackboard) {
    whirl_vel_.accel.linear.x = 0;
    whirl_vel_.accel.linear.y = 0;
    whirl_vel_.accel.linear.z = 0;

    whirl_vel_.accel.angular.x = 0;
    whirl_vel_.accel.angular.y = 0;
    whirl_vel_.accel.angular.z = 0;

    if (!LoadParam(proto_file_path)) {
      RCLCPP_ERROR(rclcpp::get_logger("escape_behavior"), "%s can't open file", __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      if (blackboard_->IsEnemyDetected()) {
        geometry_msgs::msg::PoseStamped enemy = blackboard_->GetEnemy();
        float goal_yaw = 0, goal_x = 0, goal_y = 0;
        unsigned int goal_cell_x = 0, goal_cell_y = 0;
        unsigned int enemy_cell_x = 0, enemy_cell_y = 0;

        std::random_device rd;
        std::mt19937 gen(rd());

        auto robot_map_pose = blackboard_->GetRobotMapPose();
        float x_min = 0, x_max = 0;
        if (enemy.pose.position.x < left_x_limit_) {
          x_min = right_random_min_x_;
          x_max = right_random_max_x_;

        } else if (enemy.pose.position.x > right_x_limit_) {
          x_min = left_random_min_x_;
          x_max = left_random_max_x_;

        } else {
          if ((robot_x_limit_ - robot_map_pose.pose.position.x) >= 0) {
            x_min = left_random_min_x_;
            x_max = left_random_max_x_;

          } else {
            x_min = right_random_min_x_;
            x_max = right_random_max_x_;
          }
        }

        std::uniform_real_distribution<float> x_uni_dis(x_min, x_max);
        std::uniform_real_distribution<float> y_uni_dis(0, 5);

        auto get_enemy_cell = blackboard_->GetCostMap2D()->World2Map(
            enemy.pose.position.x, enemy.pose.position.y, enemy_cell_x, enemy_cell_y);

        if (!get_enemy_cell) {
          chassis_executor_->Execute(whirl_vel_);
          return;
        }

        while (true) {
          goal_x = x_uni_dis(gen);
          goal_y = y_uni_dis(gen);
          auto get_goal_cell = blackboard_->GetCostMap2D()->World2Map(goal_x, goal_y, goal_cell_x,
                                                                       goal_cell_y);

          if (!get_goal_cell) {
            continue;
          }

          auto index = blackboard_->GetCostMap2D()->GetIndex(goal_cell_x, goal_cell_y);
          if (blackboard_->GetCharMap()[index] >= 253) {
            continue;
          }

          unsigned int obstacle_count = 0;
          for (FastLineIterator line(goal_cell_x, goal_cell_y, enemy_cell_x, enemy_cell_y);
               line.IsValid(); line.Advance()) {
            auto point_cost =
                blackboard_->GetCostMap2D()->GetCost(static_cast<unsigned int>(line.GetX()),
                                                      static_cast<unsigned int>(line.GetY()));

            if (point_cost > 253) {
              obstacle_count++;
            }
          }

          if (obstacle_count > 5) {
            break;
          }
        }
        Eigen::Vector2d pose_to_enemy(enemy.pose.position.x - robot_map_pose.pose.position.x,
                                      enemy.pose.position.y - robot_map_pose.pose.position.y);
        goal_yaw =
            static_cast<float>(std::atan2(pose_to_enemy.coeffRef(1), pose_to_enemy.coeffRef(0)));
        auto quaternion = QuaternionFromRollPitchYaw(0, 0, goal_yaw);

        geometry_msgs::msg::PoseStamped escape_goal;
        escape_goal.header.frame_id = "map";
        escape_goal.header.stamp = blackboard_->Now();
        escape_goal.pose.position.x = goal_x;
        escape_goal.pose.position.y = goal_y;
        escape_goal.pose.orientation = quaternion;
        chassis_executor_->Execute(escape_goal);
      } else {
        chassis_executor_->Execute(whirl_vel_);
      }
    }
  }

  void Cancel() { chassis_executor_->Cancel(); }

  BehaviorState Update() { return chassis_executor_->Update(); }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    left_x_limit_ = decision_config.escape().left_x_limit();
    right_x_limit_ = decision_config.escape().right_x_limit();
    robot_x_limit_ = decision_config.escape().robot_x_limit();
    left_random_min_x_ = decision_config.escape().left_random_min_x();
    left_random_max_x_ = decision_config.escape().left_random_max_x();
    right_random_min_x_ = decision_config.escape().right_random_min_x();
    right_random_max_x_ = decision_config.escape().right_random_max_x();

    whirl_vel_.twist.angular.z = decision_config.whirl_vel().angle_z_vel();
    whirl_vel_.twist.angular.y = decision_config.whirl_vel().angle_y_vel();
    whirl_vel_.twist.angular.x = decision_config.whirl_vel().angle_x_vel();

    return true;
  }

  ~EscapeBehavior() = default;

 private:
  ChassisExecutor *const chassis_executor_;

  float left_x_limit_, right_x_limit_;
  float robot_x_limit_;
  float left_random_min_x_, left_random_max_x_;
  float right_random_min_x_, right_random_max_x_;

  Blackboard *const blackboard_;

  roborts_msgs::msg::TwistAccel whirl_vel_;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_ESCAPEBEHAVIOR_H
