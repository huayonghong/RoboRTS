/***************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 ***************************************************************************/

#include "local_planner/odom_info.h"

namespace roborts_local_planner {

OdomInfo::OdomInfo(std::string topic) : topic_(std::move(topic)) {}

void OdomInfo::SetNode(const std::shared_ptr<rclcpp::Node> &node) {
  node_ = node;
  SetTopic(topic_);
}

void OdomInfo::OdomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  odom_.child_frame_id = msg->child_frame_id;
}

void OdomInfo::GetVel(geometry_msgs::msg::Twist &twist) {
  std::lock_guard<std::mutex> lock(mutex_);
  twist.linear.x = odom_.twist.twist.linear.x;
  twist.linear.y = odom_.twist.twist.linear.y;
  twist.angular.z = odom_.twist.twist.angular.z;
}

void OdomInfo::SetTopic(std::string topic) {
  topic_ = std::move(topic);

  auto n = node_.lock();

  sub_.reset();
  if (!n || topic_.empty()) {
    return;
  }

  using std::placeholders::_1;
  sub_ =
      n->create_subscription<nav_msgs::msg::Odometry>(
          topic_, 1, std::bind(&OdomInfo::OdomCB, this, _1));
}

}  // namespace roborts_local_planner
