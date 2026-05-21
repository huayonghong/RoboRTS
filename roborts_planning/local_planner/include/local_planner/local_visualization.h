/***************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 ***************************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H

#include <iterator>
#include <memory>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include "timed_elastic_band/teb_vertex_console.h"

namespace roborts_local_planner {

class LocalVisualization {
 public:
  LocalVisualization();

  LocalVisualization(std::weak_ptr<rclcpp::Node> node,
                     const std::string &visualize_frame);

  explicit LocalVisualization(const std::shared_ptr<rclcpp::Node> &node,
                              const std::string &visualize_frame);

  void Initialization(std::weak_ptr<rclcpp::Node> node,
                     const std::string &visualize_frame);

  void PublishLocalPlan(const TebVertexConsole &vertex_console) const;

 protected:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_planner_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;

  std::weak_ptr<rclcpp::Node> node_;
  std::string visual_frame_ = "map";
  bool initialized_ = false;
};

typedef std::shared_ptr<LocalVisualization> LocalVisualizationPtr;

}  // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H
