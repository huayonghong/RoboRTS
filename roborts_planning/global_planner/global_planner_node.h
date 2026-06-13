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

#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <roborts_msgs/action/global_planner.hpp>

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "state/node_state.h"
#include "costmap/costmap_interface.h"
#include "global_planner_base.h"
#include "proto/global_planner_config.pb.h"
#include "global_planner_algorithms.h"

namespace roborts_global_planner{

class GlobalPlannerNode : public rclcpp::Node {
 public:
  using GlobalPlannerAction = roborts_msgs::action::GlobalPlanner;
  using GoalHandleGlobalPlanner = rclcpp_action::ServerGoalHandle<GlobalPlannerAction>;

  typedef std::shared_ptr<roborts_costmap::CostmapInterface> CostmapPtr;
  typedef std::unique_ptr<GlobalPlannerBase> GlobalPlannerPtr;

  GlobalPlannerNode();
  ~GlobalPlannerNode();

  bool InitializeNode();

 private:
  roborts_common::ErrorInfo Init();

  rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const GlobalPlannerAction::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleGlobalPlanner> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandleGlobalPlanner> goal_handle);
  void Execute(const std::shared_ptr<GoalHandleGlobalPlanner> goal_handle);

  void SetNodeState(roborts_common::NodeState node_state);
  roborts_common::NodeState GetNodeState();

  void SetErrorInfo(roborts_common::ErrorInfo error_info);
  roborts_common::ErrorInfo GetErrorInfo();

  geometry_msgs::msg::PoseStamped GetGoal();
  void SetGoal(const geometry_msgs::msg::PoseStamped &goal);
  void StartPlanning();
  void StopPlanning();

  void PlanThread();
  double GetDistance(const geometry_msgs::msg::PoseStamped &pose1,
                     const geometry_msgs::msg::PoseStamped &pose2);
  double GetAngle(const geometry_msgs::msg::PoseStamped &pose1,
                  const geometry_msgs::msg::PoseStamped &pose2);

  void PathVisualization(const std::vector<geometry_msgs::msg::PoseStamped> &path);

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Server<GlobalPlannerAction>::SharedPtr action_server_;

  GlobalPlannerPtr global_planner_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  CostmapPtr costmap_ptr_;
  std::string selected_algorithm_;

  geometry_msgs::msg::PoseStamped goal_;
  std::mutex goal_mtx_;
  bool pause_{false};

  nav_msgs::msg::Path path_;
  bool new_path_{false};

  std::thread plan_thread_;
  std::condition_variable plan_condition_;
  std::mutex plan_mutex_;

  roborts_common::NodeState node_state_{roborts_common::NodeState::IDLE};
  std::mutex node_state_mtx_;

  roborts_common::ErrorInfo error_info_;
  std::mutex error_info_mtx_;

  std::chrono::microseconds cycle_duration_{0};
  int max_retries_{0};
  double goal_distance_tolerance_{0};
  double goal_angle_tolerance_{0};
};

} //namespace roborts_global_planner
#endif //ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
