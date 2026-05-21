/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "costmap_interface.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_costmap");

  tf2_ros::Buffer tf_buffer(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer, node);

  std::string local_map = ament_index_cpp::get_package_share_directory("roborts_costmap") +
                          "/config/costmap_parameter_config_for_local_plan.prototxt";

  roborts_costmap::CostmapInterface costmap_interface("map", node, tf_buffer, local_map);

  (void)tf_listener;
  (void)costmap_interface;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
