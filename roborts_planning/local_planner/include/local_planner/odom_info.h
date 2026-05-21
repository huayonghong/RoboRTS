/***************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 ***************************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_ODOM_INFO_H

#include <string>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace roborts_local_planner {

class OdomInfo {
 public:
  explicit OdomInfo(std::string topic = "");
  ~OdomInfo() {}

  void SetNode(const std::shared_ptr<rclcpp::Node> &node);

  void OdomCB(const nav_msgs::msg::Odometry::SharedPtr msg);

  void GetVel(geometry_msgs::msg::Twist &twist);

  void SetTopic(std::string topic);

  std::string GetTopic() const { return topic_; }

 private:
  std::string topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

  nav_msgs::msg::Odometry odom_;

  std::mutex mutex_;
  std::weak_ptr<rclcpp::Node> node_;
};

}  // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
