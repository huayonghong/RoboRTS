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

#include "chassis_executor.h"

#include <chrono>
#include <functional>
#include <thread>

namespace roborts_decision {

ChassisExecutor::ChassisExecutor(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      execution_mode_(ExcutionMode::IDLE_MODE),
      execution_state_(BehaviorState::IDLE) {
  global_planner_client_ =
      rclcpp_action::create_client<GlobalPlannerAction>(*node_, "/global_planner_node_action");
  local_planner_client_ =
      rclcpp_action::create_client<LocalPlannerAction>(*node_, "/local_planner_node_action");

  cmd_vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1));
  cmd_vel_acc_pub_ =
      node_->create_publisher<roborts_msgs::msg::TwistAccel>("cmd_vel_acc", rclcpp::QoS(100));

  while (!global_planner_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for global planner action server...");
    rclcpp::spin_some(node_);
  }
  RCLCPP_INFO(node_->get_logger(), "Global planer server start!");

  while (!local_planner_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for local planner action server...");
    rclcpp::spin_some(node_);
  }
  RCLCPP_INFO(node_->get_logger(), "Local planer server start!");
}

void ChassisExecutor::Execute(const geometry_msgs::msg::PoseStamped &goal) {
  execution_mode_ = ExcutionMode::GOAL_MODE;

  GlobalPlannerAction::Goal cmd;
  cmd.command = 0;
  cmd.goal = goal;

  auto opts = rclcpp_action::Client<GlobalPlannerAction>::SendGoalOptions();
  opts.goal_response_callback = [this](GoalHandleGlobal::SharedPtr gh) {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    global_goal_handle_ = std::move(gh);
  };
  opts.feedback_callback =
      std::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, std::placeholders::_1,
                std::placeholders::_2);

  global_planner_client_->async_send_goal(cmd, opts);
}

void ChassisExecutor::Execute(const geometry_msgs::msg::Twist &twist) {
  if (execution_mode_ == ExcutionMode::GOAL_MODE) {
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_MODE;
  cmd_vel_pub_->publish(twist);
}

void ChassisExecutor::Execute(const roborts_msgs::msg::TwistAccel &twist_accel) {
  if (execution_mode_ == ExcutionMode::GOAL_MODE) {
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_WITH_ACCEL_MODE;
  cmd_vel_acc_pub_->publish(twist_accel);
}

BehaviorState ChassisExecutor::Update() {
  switch (execution_mode_) {
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::GOAL_MODE: {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      if (!global_goal_handle_) {
        execution_state_ = BehaviorState::RUNNING;
        break;
      }
      auto status = global_goal_handle_->get_status();

      switch (status) {
        using GS = action_msgs::msg::GoalStatus;
        case GS::STATUS_ACCEPTED:
        case GS::STATUS_EXECUTING:
        case GS::STATUS_CANCELING:
          execution_state_ = BehaviorState::RUNNING;
          break;
        case GS::STATUS_SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "%s : SUCCEEDED", __FUNCTION__);
          execution_state_ = BehaviorState::SUCCESS;
          break;
        case GS::STATUS_ABORTED:
        case GS::STATUS_CANCELED:
          RCLCPP_INFO(node_->get_logger(), "%s : ABORTED/CANCELED", __FUNCTION__);
          execution_state_ = BehaviorState::FAILURE;
          break;
        default:
          execution_state_ = BehaviorState::RUNNING;
          break;
      }
      break;
    }

    case ExcutionMode::SPEED_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Wrong Execution Mode");
  }
  return execution_state_;
}

void ChassisExecutor::Cancel() {
  switch (execution_mode_) {
    case ExcutionMode::IDLE_MODE:
      RCLCPP_WARN(node_->get_logger(), "Nothing to be canceled.");
      break;

    case ExcutionMode::GOAL_MODE:
      global_planner_client_->async_cancel_all_goals();
      local_planner_client_->async_cancel_all_goals();
      {
        std::lock_guard<std::mutex> lk(goal_mutex_);
        global_goal_handle_.reset();
      }
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_MODE:
      cmd_vel_pub_->publish(zero_twist_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      cmd_vel_acc_pub_->publish(zero_twist_accel_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      std::this_thread::sleep_for(std::chrono::microseconds(50000));
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Wrong Execution Mode");
  }
}

void ChassisExecutor::GlobalPlannerFeedbackCallback(
    GoalHandleGlobal::SharedPtr,
    const std::shared_ptr<const GlobalPlannerAction::Feedback> feedback) {
  if (feedback != nullptr && !feedback->path.poses.empty()) {
    LocalPlannerAction::Goal local_goal;
    local_goal.route = feedback->path;
    local_planner_client_->async_send_goal(local_goal);
  }
}

} // namespace roborts_decision
