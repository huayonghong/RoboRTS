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

#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <roborts_msgs/action/armor_detection.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "costmap/costmap_interface.h"

#include "../proto/decision.pb.h"

namespace roborts_decision {

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  typedef rclcpp_action::ClientGoalHandle<roborts_msgs::action::ArmorDetection> ArmorGoalHandle;

  explicit Blackboard(rclcpp::Node::SharedPtr node, const std::string &proto_file_path);

  ~Blackboard();

  geometry_msgs::msg::PoseStamped GetEnemy() const { return enemy_pose_; }

  bool IsEnemyDetected() const;

  void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr &goal);

  geometry_msgs::msg::PoseStamped GetGoal() const { return goal_; }

  bool IsNewGoal();

  double GetDistance(const geometry_msgs::msg::PoseStamped &pose1,
                     const geometry_msgs::msg::PoseStamped &pose2) const;

  double GetAngle(const geometry_msgs::msg::PoseStamped &pose1,
                  const geometry_msgs::msg::PoseStamped &pose2) const;

  geometry_msgs::msg::PoseStamped GetRobotMapPose();

  std::shared_ptr<CostMap> GetCostMap() { return costmap_ptr_; }

  CostMap2D *GetCostMap2D() { return costmap_2d_; }

  const unsigned char *GetCharMap() { return charmap_; }

  rclcpp::Time Now() const { return node_->now(); }

 private:
  void ArmorDetectionFeedbackCb(
      ArmorGoalHandle::SharedPtr goal_handle_unused,
      const std::shared_ptr<const roborts_msgs::action::ArmorDetection::Feedback> feedback);

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr enemy_sub_;

  geometry_msgs::msg::PoseStamped goal_;
  bool new_goal_{false};

  bool simulate_{false};

  rclcpp_action::Client<roborts_msgs::action::ArmorDetection>::SharedPtr armor_detection_client_;

  geometry_msgs::msg::PoseStamped enemy_pose_;
  bool enemy_detected_{false};

  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D *costmap_2d_{nullptr};
  unsigned char *charmap_{nullptr};

  geometry_msgs::msg::PoseStamped robot_map_pose_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_BLACKBOARD_H
