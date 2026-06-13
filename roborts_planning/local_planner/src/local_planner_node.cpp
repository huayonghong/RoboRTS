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

#include <chrono>
#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <thread>

#include "local_planner/local_planner_node.h"

namespace roborts_local_planner {

using roborts_common::NodeState;

LocalPlannerNode::LocalPlannerNode()
    : Node("local_planner_node"),
      initialized_(false),
      node_state_(roborts_common::NodeState::IDLE),
      node_error_info_(roborts_common::ErrorCode::OK),
      max_error_(5),
      frequency_(10.0) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool LocalPlannerNode::InitializeNode() {
  action_server_ = rclcpp_action::create_server<LocalPlannerAction>(
      shared_from_this(), "local_planner_node_action",
      std::bind(&LocalPlannerNode::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LocalPlannerNode::HandleCancel, this, std::placeholders::_1),
      std::bind(&LocalPlannerNode::HandleAccepted, this, std::placeholders::_1));

  if (Init().IsOK()) {
    RCLCPP_INFO(get_logger(), "local planner initialize completed.");
    return true;
  }

  RCLCPP_WARN(get_logger(), "local planner initialize failed.");
  SetNodeState(NodeState::FAILURE);
  return false;
}

LocalPlannerNode::~LocalPlannerNode() { StopPlanning(); }

roborts_common::ErrorInfo LocalPlannerNode::Init() {
  RCLCPP_INFO(get_logger(), "local planner start");
  LocalAlgorithms local_algorithms;

  std::string full_path = ament_index_cpp::get_package_share_directory("roborts_planning") +
                         "/local_planner/config/local_planner.prototxt";
  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(), &local_algorithms)) {
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
                                     "Cannot load local planner protobuf configuration file.");
  }
  selected_algorithm_ = local_algorithms.selected_algorithm();
  frequency_ = local_algorithms.frequency();

  std::string map_path = ament_index_cpp::get_package_share_directory("roborts_costmap") +
                        "/config/costmap_parameter_config_for_local_plan.prototxt";

  local_cost_ = std::make_shared<roborts_costmap::CostmapInterface>("local_costmap", shared_from_this(), *tf_buffer_,
                                                                     map_path);

  local_planner_ = roborts_common::AlgorithmFactory<LocalPlannerBase>::CreateAlgorithm(selected_algorithm_);
  if (local_planner_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "global planner algorithm instance can't be loaded");
    return roborts_common::ErrorInfo(
        roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
        "local planner algorithm instance can't be loaded");
  }

  visual_frame_ = local_cost_->GetGlobalFrameID();
  visual_ = std::make_shared<LocalVisualization>(shared_from_this(), visual_frame_);

  vel_pub_ = create_publisher<roborts_msgs::msg::TwistAccel>("/cmd_vel_acc", rclcpp::QoS(5));

  initialized_ = true;

  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

rclcpp_action::GoalResponse LocalPlannerNode::HandleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const LocalPlannerAction::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LocalPlannerNode::HandleCancel(
    const std::shared_ptr<GoalHandleLocalPlanner>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LocalPlannerNode::HandleAccepted(const std::shared_ptr<GoalHandleLocalPlanner> goal_handle) {
  std::thread{std::bind(&LocalPlannerNode::Execute, this, goal_handle)}.detach();
}

void LocalPlannerNode::Execute(const std::shared_ptr<GoalHandleLocalPlanner> goal_handle) {
  auto error_info = GetErrorInfo();
  auto node_state = GetNodeState();

  if (node_state == NodeState::FAILURE) {
    auto feedback = std::make_shared<LocalPlannerAction::Feedback>();
    auto result = std::make_shared<LocalPlannerAction::Result>();
    feedback->error_code = error_info.error_code();
    feedback->error_msg = error_info.error_msg();
    result->error_code = feedback->error_code;
    goal_handle->publish_feedback(feedback);
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "Initialization Failed, Failed to execute action!");
    return;
  }

  auto command = goal_handle->get_goal();
  geometry_msgs::msg::PoseStamped fallback_goal{};
  if (!command->route.poses.empty()) {
    fallback_goal = command->route.poses.back();
  }

  if (plan_mtx_.try_lock()) {
    local_planner_->SetPlan(command->route, fallback_goal);
    plan_mtx_.unlock();
    plan_condition_.notify_one();
  }

  RCLCPP_INFO(get_logger(), "Send Plan!");
  if (node_state == NodeState::IDLE) {
    StartPlanning();
  }

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<LocalPlannerAction::Result>();
      result->error_code = static_cast<int32_t>(roborts_common::ErrorCode::OK);
      goal_handle->canceled(result);
      StopPlanning();
      break;
    }

    node_state = GetNodeState();
    error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS ||
        node_state == NodeState::FAILURE) {
      auto feedback = std::make_shared<LocalPlannerAction::Feedback>();
      auto result = std::make_shared<LocalPlannerAction::Result>();

      if (!error_info.IsOK()) {
        feedback->error_code = error_info.error_code();
        feedback->error_msg = error_info.error_msg();
        SetErrorInfo(roborts_common::ErrorInfo::OK());
        goal_handle->publish_feedback(feedback);
      }
      if (node_state == NodeState::SUCCESS) {
        result->error_code = error_info.error_code();
        goal_handle->succeed(result);
        StopPlanning();
        break;
      }
      if (node_state == NodeState::FAILURE) {
        result->error_code = error_info.error_code();
        goal_handle->abort(result);
        StopPlanning();
        break;
      }
    }
  }
}

void LocalPlannerNode::Loop() {
  roborts_common::ErrorInfo error_info_init = local_planner_->Initialize(local_cost_, tf_buffer_, visual_);
  if (error_info_init.IsOK()) {
    RCLCPP_INFO(get_logger(), "local planner algorithm initialize completed.");
  } else {
    RCLCPP_WARN(get_logger(), "local planner algorithm initialize failed.");
    SetNodeState(NodeState::FAILURE);
    SetErrorInfo(error_info_init);
  }

  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  int error_count = 0;

  while (GetNodeState() == NodeState::RUNNING) {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin = std::chrono::steady_clock::now();

    roborts_common::ErrorInfo error_info = local_planner_->ComputeVelocityCommands(cmd_vel_);

    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);
    int need_time = static_cast<int>(1000.0 / frequency_);
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;

    if (sleep_time <= std::chrono::milliseconds(0)) {
      sleep_time = std::chrono::milliseconds(0);
    }

    if (error_info.IsOK()) {
      error_count = 0;
      vel_pub_->publish(cmd_vel_);
      if (local_planner_->IsGoalReached()) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (error_count > max_error_ && max_error_ > 0) {
      RCLCPP_WARN(get_logger(), "Can not finish plan with max retries( %d  )", max_error_);
      error_info =
          roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
      SetNodeState(NodeState::FAILURE);
    } else {
      error_count++;
      RCLCPP_ERROR(get_logger(), "Can not get cmd_vel for once. %s error count:  %d",
                   error_info.error_msg().c_str(), error_count);
    }

    SetErrorInfo(error_info);
  }

  cmd_vel_.twist.linear.x = 0;
  cmd_vel_.twist.linear.y = 0;
  cmd_vel_.twist.angular.z = 0;
  cmd_vel_.accel.linear.x = 0;
  cmd_vel_.accel.linear.y = 0;
  cmd_vel_.accel.angular.z = 0;

  vel_pub_->publish(cmd_vel_);
}

void LocalPlannerNode::SetErrorInfo(const roborts_common::ErrorInfo error_info) {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  node_error_info_ = error_info;
}

void LocalPlannerNode::SetNodeState(const roborts_common::NodeState &node_state) {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}

roborts_common::NodeState LocalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}

roborts_common::ErrorInfo LocalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  return node_error_info_;
}

void LocalPlannerNode::StartPlanning() {
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }

  SetNodeState(roborts_common::NodeState::RUNNING);
  local_planner_thread_ = std::thread(std::bind(&LocalPlannerNode::Loop, this));
}

void LocalPlannerNode::StopPlanning() {
  SetNodeState(roborts_common::NodeState::IDLE);
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }
}

void LocalPlannerNode::AlgorithmCB(const roborts_common::ErrorInfo &algorithm_error_info) {
  SetErrorInfo(algorithm_error_info);
}

}  // namespace roborts_local_planner

static void SignalHandler(int signal_number) {
  (void)signal_number;
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roborts_local_planner::LocalPlannerNode>();
  if (!node->InitializeNode()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();

  node->StopPlanning();

  rclcpp::shutdown();
  return 0;
}
