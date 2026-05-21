/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DECISION_CHASSIS_EXECUTOR_H
#define ROBORTS_DECISION_CHASSIS_EXECUTOR_H

#include <mutex>
#include <memory>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <roborts_msgs/action/global_planner.hpp>
#include <roborts_msgs/action/local_planner.hpp>
#include <roborts_msgs/msg/twist_accel.hpp>

#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {

class ChassisExecutor {
 public:
  using GlobalPlannerAction = roborts_msgs::action::GlobalPlanner;
  using LocalPlannerAction = roborts_msgs::action::LocalPlanner;
  using GoalHandleGlobal = rclcpp_action::ClientGoalHandle<GlobalPlannerAction>;

  enum class ExcutionMode {
    IDLE_MODE,
    GOAL_MODE,
    SPEED_MODE,
    SPEED_WITH_ACCEL_MODE
  };

  explicit ChassisExecutor(rclcpp::Node::SharedPtr node);
  ~ChassisExecutor() = default;

  void Execute(const geometry_msgs::msg::PoseStamped &goal);
  void Execute(const geometry_msgs::msg::Twist &twist);
  void Execute(const roborts_msgs::msg::TwistAccel &twist_accel);

  BehaviorState Update();
  void Cancel();

 private:
  void GlobalPlannerFeedbackCallback(
      GoalHandleGlobal::SharedPtr,
      const std::shared_ptr<const GlobalPlannerAction::Feedback> feedback);

  rclcpp::Node::SharedPtr node_;

  ExcutionMode execution_mode_;
  BehaviorState execution_state_;

  std::mutex goal_mutex_;
  rclcpp_action::Client<GlobalPlannerAction>::SharedPtr global_planner_client_;
  rclcpp_action::Client<LocalPlannerAction>::SharedPtr local_planner_client_;
  GoalHandleGlobal::SharedPtr global_goal_handle_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist zero_twist_;

  rclcpp::Publisher<roborts_msgs::msg::TwistAccel>::SharedPtr cmd_vel_acc_pub_;
  roborts_msgs::msg::TwistAccel zero_twist_accel_;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_CHASSIS_EXECUTOR_H
