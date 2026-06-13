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

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <roborts_msgs/action/global_planner.hpp>

#include "state/error_code.h"

using roborts_common::ErrorCode;

using GlobalPlanner = roborts_msgs::action::GlobalPlanner;
using GoalHandleGlobalPlanner =
    rclcpp_action::ClientGoalHandle<GlobalPlanner>;

class GlobalPlannerTest : public rclcpp::Node {
 public:
  explicit GlobalPlannerTest()
      : Node("global_planner_test") {
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10,
        std::bind(&GlobalPlannerTest::GoalCallback, this,
                  std::placeholders::_1));
    client_ = rclcpp_action::create_client<GlobalPlanner>(
        get_node_base_interface(), get_node_graph_interface(),
        get_node_logging_interface(), get_node_waitables_interface(),
        "/global_planner_node_action");
    timer_ =
        create_wall_timer(std::chrono::milliseconds(100),
                         std::bind(&GlobalPlannerTest::TryConnectTimer, this));
  }

 private:
  void TryConnectTimer() {
    if (client_ready_) {
      return;
    }
    if (!client_->action_server_is_ready()) {
      return;
    }
    client_ready_ = true;
    RCLCPP_INFO(get_logger(),
                "Connected to global_planner_node_action.");
  }

  void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
    if (!client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(),
                  "Global planner action server not ready.");
      return;
    }
    GlobalPlanner::Goal cmd;
    cmd.command = 0;
    cmd.goal = *goal_msg;
    auto opts = rclcpp_action::Client<GlobalPlanner>::SendGoalOptions();
    opts.goal_response_callback =
        [](std::shared_ptr<GoalHandleGlobalPlanner> gh) {
          (void)gh;
          RCLCPP_INFO(rclcpp::get_logger("planning"),
                     "Goal accepted by global planner.");
        };
    opts.feedback_callback =
        [](GoalHandleGlobalPlanner::SharedPtr,
           const std::shared_ptr<const GlobalPlanner::Feedback> feedback) {
          if (feedback->error_code != ErrorCode::OK) {
            RCLCPP_INFO(rclcpp::get_logger("planning"), "%s",
                       feedback->error_msg.c_str());
          }
          if (!feedback->path.poses.empty()) {
            RCLCPP_INFO(rclcpp::get_logger("planning"), "Get Path!");
          }
        };
    opts.result_callback =
        [&](const GoalHandleGlobalPlanner::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(),
                       "Goal finished with SUCCESS.");
          } else {
            RCLCPP_INFO(get_logger(),
                       "Goal finished with terminal state.");
          }
        };
    client_->async_send_goal(cmd, opts);
  }

  bool client_ready_{false};
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp_action::Client<GlobalPlanner>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerTest>());
  rclcpp::shutdown();
  return 0;
}
