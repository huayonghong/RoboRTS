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
#include <string>

#include <rclcpp/rclcpp.hpp>

#ifndef ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H
#define ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H

namespace roborts_localization {

// Get parameters from ROS2 parameter server (declare + get pattern)
struct LocalizationConfig {
  void GetParam(rclcpp::Node::SharedPtr node) {
    node->declare_parameter<std::string>("odom_frame", std::string("odom"));
    node->declare_parameter<std::string>("base_frame", std::string("base_link"));
    node->declare_parameter<std::string>("global_frame", std::string("map"));
    node->declare_parameter<std::string>("laser_topic_name", std::string("scan"));
    node->declare_parameter<std::string>("map_topic_name", std::string("map"));
    node->declare_parameter<std::string>(
        "init_pose_topic_name", std::string("initialpose"));
    node->declare_parameter<double>("transform_tolerance", 0.1);
    node->declare_parameter<double>("initial_pose_x", 1.0);
    node->declare_parameter<double>("initial_pose_y", 1.0);
    node->declare_parameter<double>("initial_pose_a", 0.0);
    node->declare_parameter<double>("initial_cov_xx", 0.1);
    node->declare_parameter<double>("initial_cov_yy", 0.1);
    node->declare_parameter<double>("initial_cov_aa", 0.1);
    node->declare_parameter<bool>("enable_uwb", false);
    node->declare_parameter<std::string>("uwb_frame_id", std::string("uwb"));
    node->declare_parameter<std::string>("uwb_topic_name", std::string("uwb"));
    node->declare_parameter<bool>("use_sim_uwb", false);
    node->declare_parameter<int>("uwb_correction_frequency", 20);
    node->declare_parameter<bool>("publish_visualize", true);

    node->get_parameter("odom_frame", odom_frame_id);
    node->get_parameter("base_frame", base_frame_id);
    node->get_parameter("global_frame", global_frame_id);
    node->get_parameter("laser_topic_name", laser_topic_name);
    node->get_parameter("map_topic_name", map_topic_name);
    node->get_parameter("init_pose_topic_name", init_pose_topic_name);
    node->get_parameter("transform_tolerance", transform_tolerance);
    node->get_parameter("initial_pose_x", initial_pose_x);
    node->get_parameter("initial_pose_y", initial_pose_y);
    node->get_parameter("initial_pose_a", initial_pose_a);
    node->get_parameter("initial_cov_xx", initial_cov_xx);
    node->get_parameter("initial_cov_yy", initial_cov_yy);
    node->get_parameter("initial_cov_aa", initial_cov_aa);
    node->get_parameter("enable_uwb", enable_uwb);
    node->get_parameter("uwb_frame_id", uwb_frame_id);
    node->get_parameter("uwb_topic_name", uwb_topic_name);
    node->get_parameter("use_sim_uwb", use_sim_uwb);
    node->get_parameter("uwb_correction_frequency",
                        uwb_correction_frequency);
    node->get_parameter("publish_visualize", publish_visualize);
  }
  std::string odom_frame_id;
  std::string base_frame_id;
  std::string global_frame_id;

  std::string laser_topic_name;
  std::string map_topic_name;
  std::string init_pose_topic_name;

  double transform_tolerance;

  double initial_pose_x;
  double initial_pose_y;
  double initial_pose_a;
  double initial_cov_xx;
  double initial_cov_yy;
  double initial_cov_aa;

  bool publish_visualize;

  bool enable_uwb;
  std::string uwb_frame_id;
  std::string uwb_topic_name;
  bool use_sim_uwb;
  int uwb_correction_frequency;

};

}  // namespace roborts_localization

#endif  // ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H
