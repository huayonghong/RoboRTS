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

#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "global_planner_node.h"

namespace roborts_global_planner{

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;
using roborts_common::NodeState;

GlobalPlannerNode::GlobalPlannerNode()
    : Node("global_planner_node"),
      new_path_(false),
      pause_(false),
      node_state_(NodeState::IDLE),
      error_info_(ErrorCode::OK) {

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool GlobalPlannerNode::InitializeNode() {
  action_server_ = rclcpp_action::create_server<GlobalPlannerAction>(
      shared_from_this(),
      "global_planner_node_action",
      std::bind(&GlobalPlannerNode::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GlobalPlannerNode::HandleCancel, this, std::placeholders::_1),
      std::bind(&GlobalPlannerNode::HandleAccepted, this, std::placeholders::_1));

  if (Init().IsOK()) {
    RCLCPP_INFO(get_logger(), "Global planner initialization completed.");
    StartPlanning();
    return true;
  }

  RCLCPP_ERROR(get_logger(), "Initialization failed.");
  SetNodeState(NodeState::FAILURE);
  return false;
}

ErrorInfo GlobalPlannerNode::Init() {
  GlobalPlannerConfig global_planner_config;
  const std::string full_path =
      ament_index_cpp::get_package_share_directory("roborts_planning") +
      "/global_planner/config/global_planner_config.prototxt";
  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                           &global_planner_config)) {
    RCLCPP_ERROR(get_logger(),
                 "Cannot load global planner protobuf configuration file.");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "Cannot load global planner protobuf configuration file.");
  }

  selected_algorithm_ = global_planner_config.selected_algorithm();
  cycle_duration_ = std::chrono::microseconds((int)(1e6 / global_planner_config.frequency()));
  max_retries_ = global_planner_config.max_retries();
  goal_distance_tolerance_ = global_planner_config.goal_distance_tolerance();
  goal_angle_tolerance_ = global_planner_config.goal_angle_tolerance();

  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);

  const std::string map_path =
      ament_index_cpp::get_package_share_directory("roborts_costmap") +
      "/config/costmap_parameter_config_for_global_plan.prototxt";
  costmap_ptr_ = std::make_shared<roborts_costmap::CostmapInterface>(
      "global_costmap", shared_from_this(), *tf_buffer_, map_path);

  global_planner_ptr_ =
      roborts_common::AlgorithmFactory<GlobalPlannerBase, CostmapPtr>::CreateAlgorithm(
          selected_algorithm_, costmap_ptr_);
  if (global_planner_ptr_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "global planner algorithm instance can't be loaded");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "global planner algorithm instance can't be loaded");
  }

  path_.header.frame_id = costmap_ptr_->GetGlobalFrameID();
  return ErrorInfo(ErrorCode::OK);
}

rclcpp_action::GoalResponse GlobalPlannerNode::HandleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GlobalPlannerAction::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlannerNode::HandleCancel(
    const std::shared_ptr<GoalHandleGlobalPlanner>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlannerNode::HandleAccepted(const std::shared_ptr<GoalHandleGlobalPlanner> goal_handle) {
  std::thread{std::bind(&GlobalPlannerNode::Execute, this, goal_handle)}.detach();
}

void GlobalPlannerNode::Execute(const std::shared_ptr<GoalHandleGlobalPlanner> goal_handle) {
  RCLCPP_INFO(get_logger(), "Received a Goal from client!");
  auto goal_msg = goal_handle->get_goal();
  SetGoal(goal_msg->goal);

  if (GetNodeState() != NodeState::RUNNING) {
    SetNodeState(NodeState::RUNNING);
  }

  {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.notify_one();
  }

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<GlobalPlannerAction::Result>();
      result->error_code = static_cast<int32_t>(GetErrorInfo().error_code());
      goal_handle->canceled(result);
      SetNodeState(NodeState::IDLE);
      RCLCPP_INFO(get_logger(), "Cancel!");
      return;
    }

    NodeState node_state = GetNodeState();
    ErrorInfo error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING ||
        node_state == NodeState::SUCCESS ||
        node_state == NodeState::FAILURE) {
      auto feedback = std::make_shared<GlobalPlannerAction::Feedback>();
      auto result = std::make_shared<GlobalPlannerAction::Result>();

      if (!error_info.IsOK() || new_path_) {
        if (!error_info.IsOK()) {
          feedback->error_code = error_info.error_code();
          feedback->error_msg = error_info.error_msg();
          SetErrorInfo(ErrorInfo::OK());
        }
        if (new_path_) {
          feedback->path = path_;
          new_path_ = false;
        }
        goal_handle->publish_feedback(feedback);
      }

      if (node_state == NodeState::SUCCESS) {
        result->error_code = error_info.error_code();
        goal_handle->succeed(result);
        SetNodeState(NodeState::IDLE);
        break;
      } else if (node_state == NodeState::FAILURE) {
        result->error_code = error_info.error_code();
        goal_handle->abort(result);
        SetNodeState(NodeState::IDLE);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

NodeState GlobalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  return node_state_;
}

void GlobalPlannerNode::SetNodeState(NodeState node_state) {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  node_state_ = node_state;
}

ErrorInfo GlobalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> error_info_lock(error_info_mtx_);
  return error_info_;
}

void GlobalPlannerNode::SetErrorInfo(ErrorInfo error_info) {
  std::lock_guard<std::mutex> node_state_lock(error_info_mtx_);
  error_info_ = error_info;
}

geometry_msgs::msg::PoseStamped GlobalPlannerNode::GetGoal() {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  return goal_;
}

void GlobalPlannerNode::SetGoal(const geometry_msgs::msg::PoseStamped &goal) {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  goal_ = goal;
}

void GlobalPlannerNode::StartPlanning() {
  SetNodeState(NodeState::IDLE);
  plan_thread_ = std::thread(&GlobalPlannerNode::PlanThread, this);
}

void GlobalPlannerNode::StopPlanning() {
  SetNodeState(NodeState::RUNNING);
  if (plan_thread_.joinable()) {
    plan_thread_.join();
  }
}

void GlobalPlannerNode::PlanThread() {
  RCLCPP_INFO(get_logger(), "Plan thread start!");
  geometry_msgs::msg::PoseStamped current_start;
  geometry_msgs::msg::PoseStamped current_goal;
  std::vector<geometry_msgs::msg::PoseStamped> current_path;
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  ErrorInfo error_info;
  int retries = 0;
  while (rclcpp::ok()) {
    RCLCPP_INFO(get_logger(), "Wait to plan!");
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    while (GetNodeState() != NodeState::RUNNING) {
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    RCLCPP_INFO(get_logger(), "Go on planning!");

    auto start_time = std::chrono::steady_clock::now();

    {
      std::unique_lock<roborts_costmap::Costmap2D::mutex_t> lock(
          *(costmap_ptr_->GetCostMap()->GetMutex()));
      bool error_set = false;
      while (!costmap_ptr_->GetRobotPose(current_start)) {
        if (!error_set) {
          RCLCPP_ERROR(get_logger(), "Get Robot Pose Error.");
          SetErrorInfo(ErrorInfo(ErrorCode::GP_GET_POSE_ERROR, "Get Robot Pose Error."));
          error_set = true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }

      current_goal = GetGoal();

      if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID()) {
        current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal);
        SetGoal(current_goal);
      }

      error_info =
          global_planner_ptr_->Plan(current_start, current_goal, current_path);
    }

    if (error_info.IsOK()) {
      retries = 0;
      PathVisualization(current_path);

      current_goal = current_path.back();
      SetGoal(current_goal);

      if (GetDistance(current_start, current_goal) < goal_distance_tolerance_ &&
          GetAngle(current_start, current_goal) < goal_angle_tolerance_) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (max_retries_ > 0 && retries > max_retries_) {
      RCLCPP_ERROR(get_logger(),
                   "Can not get plan with max retries( %d )", max_retries_);
      error_info = ErrorInfo(ErrorCode::GP_MAX_RETRIES_FAILURE, "Over max retries.");
      SetNodeState(NodeState::FAILURE);
      retries = 0;
    } else if (error_info == ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR)) {
      RCLCPP_ERROR(get_logger(), "Current goal is not valid!");
      SetNodeState(NodeState::FAILURE);
      retries = 0;
    } else {
      retries++;
      RCLCPP_ERROR(get_logger(),
                   "Can not get plan for once. %s",
                   error_info.error_msg().c_str());
    }

    SetErrorInfo(error_info);

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    sleep_time = cycle_duration_ - execution_duration;

    if (sleep_time <= std::chrono::microseconds(0)) {
      RCLCPP_ERROR(get_logger(),
                   "The time planning once is %ld beyond the expected time %ld",
                   execution_duration.count(),
                   cycle_duration_.count());
      sleep_time = std::chrono::microseconds(0);
      SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }
  }

  RCLCPP_INFO(get_logger(), "Plan thread terminated!");
}

void GlobalPlannerNode::PathVisualization(
    const std::vector<geometry_msgs::msg::PoseStamped> &path) {
  path_.poses = path;
  path_pub_->publish(path_);
  new_path_ = true;
}

double GlobalPlannerNode::GetDistance(const geometry_msgs::msg::PoseStamped &pose1,
                                      const geometry_msgs::msg::PoseStamped &pose2) {
  const geometry_msgs::msg::Point point1 = pose1.pose.position;
  const geometry_msgs::msg::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double GlobalPlannerNode::GetAngle(const geometry_msgs::msg::PoseStamped &pose1,
                                   const geometry_msgs::msg::PoseStamped &pose2) {
  const double yaw1 =
      tf2::getYaw(pose1.pose.orientation);  // NOLINT: tf2 yaw helper
  const double yaw2 = tf2::getYaw(pose2.pose.orientation);
  double dy = yaw1 - yaw2;
  while (dy > M_PI) dy -= 2 * M_PI;
  while (dy < -M_PI) dy += 2 * M_PI;
  return std::fabs(dy);
}

GlobalPlannerNode::~GlobalPlannerNode() {
  StopPlanning();
}

} //namespace roborts_global_planner

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roborts_global_planner::GlobalPlannerNode>();
  if (!node->InitializeNode()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
