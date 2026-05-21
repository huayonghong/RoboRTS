/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_NODE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_NODE_H

#include <string>
#include <thread>
#include <memory>
#include <mutex>
#include <functional>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <roborts_msgs/action/local_planner.hpp>
#include <roborts_msgs/msg/twist_accel.hpp>

#include "io/io.h"
#include "state/error_code.h"
#include "state/node_state.h"
#include "alg_factory/algorithm_factory.h"

#include "costmap/costmap_interface.h"
#include "local_planner/proto/local_planner.pb.h"
#include "local_planner/local_planner_base.h"
#include "local_planner/local_visualization.h"
#include "local_planner/local_planner_algorithms.h"

namespace roborts_local_planner {

class LocalPlannerNode : public rclcpp::Node {
 public:
  using LocalPlannerAction = roborts_msgs::action::LocalPlanner;
  using GoalHandleLocalPlanner = rclcpp_action::ServerGoalHandle<LocalPlannerAction>;

  LocalPlannerNode();

  ~LocalPlannerNode();

  roborts_common::ErrorInfo Init();

  void Loop();

  void AlgorithmCB(const roborts_common::ErrorInfo &algorithm_error_info);

  void StartPlanning();

  void StopPlanning();

  void SetNodeState(const roborts_common::NodeState &node_state);

  roborts_common::NodeState GetNodeState();

  void SetErrorInfo(const roborts_common::ErrorInfo error_info);

  roborts_common::ErrorInfo GetErrorInfo();

 private:
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const LocalPlannerAction::Goal> goal);

  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleLocalPlanner> goal_handle);

  void HandleAccepted(const std::shared_ptr<GoalHandleLocalPlanner> goal_handle);

  void Execute(const std::shared_ptr<GoalHandleLocalPlanner> goal_handle);

  std::thread local_planner_thread_;
  rclcpp_action::Server<LocalPlannerAction>::SharedPtr action_server_;

  std::unique_ptr<LocalPlannerBase> local_planner_;
  std::mutex node_state_mtx_;
  std::mutex node_error_info_mtx_;
  std::mutex plan_mtx_;

  roborts_common::NodeState node_state_;
  roborts_common::ErrorInfo node_error_info_;

  std::shared_ptr<roborts_costmap::CostmapInterface> local_cost_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  bool initialized_;

  std::string selected_algorithm_;

  roborts_msgs::msg::TwistAccel cmd_vel_;

  LocalVisualizationPtr visual_;
  std::string visual_frame_;

  rclcpp::Publisher<roborts_msgs::msg::TwistAccel>::SharedPtr vel_pub_;

  geometry_msgs::msg::PoseStamped local_goal_;

  int max_error_;

  std::condition_variable plan_condition_;

  std::mutex plan_mutex_;

  double frequency_;
};

}  // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_LOCAL_PLANNER_NODE_H
