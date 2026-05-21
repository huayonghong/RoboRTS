/***************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 ***************************************************************************/

#include "local_planner/local_visualization.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace roborts_local_planner {

LocalVisualization::LocalVisualization() {}

LocalVisualization::LocalVisualization(std::weak_ptr<rclcpp::Node> node,
                                       const std::string &visualize_frame) {
  Initialization(node, visualize_frame);
}

LocalVisualization::LocalVisualization(const std::shared_ptr<rclcpp::Node> &node,
                                       const std::string &visualize_frame) {
  Initialization(std::weak_ptr<rclcpp::Node>(node), visualize_frame);
}

void LocalVisualization::Initialization(std::weak_ptr<rclcpp::Node> node,
                                        const std::string &visualize_frame) {
  if (initialized_) {
    return;
  }
  node_ = node;
  visual_frame_ = visualize_frame;
  auto nh = node_.lock();
  if (nh) {
    local_planner_pub_ =
        nh->create_publisher<nav_msgs::msg::Path>("trajectory", rclcpp::QoS(1));
    pose_pub_ =
        nh->create_publisher<geometry_msgs::msg::PoseArray>("pose", rclcpp::QoS(1));
  }
  initialized_ = true;
}

void LocalVisualization::PublishLocalPlan(const TebVertexConsole &vertex_console) const {
  auto nh = node_.lock();
  if (!nh || !local_planner_pub_) {
    return;
  }

  nav_msgs::msg::Path local_plan;
  local_plan.header.frame_id = visual_frame_;
  local_plan.header.stamp = nh->now();

  for (int i = 0; i < vertex_console.SizePoses(); ++i) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = local_plan.header.frame_id;
    pose_stamped.header.stamp = local_plan.header.stamp;
    pose_stamped.pose.position.x = vertex_console.Pose(i).GetPosition().coeffRef(0);
    pose_stamped.pose.position.y = vertex_console.Pose(i).GetPosition().coeffRef(1);
    pose_stamped.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, vertex_console.Pose(i).GetTheta());
    pose_stamped.pose.orientation = tf2::toMsg(q);
    local_plan.poses.push_back(pose_stamped);
  }
  local_planner_pub_->publish(local_plan);
}

}  // namespace roborts_local_planner
