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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef ROBORTS_COSTMAP_COSTMAP_INTERFACE_H
#define ROBORTS_COSTMAP_COSTMAP_INTERFACE_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <thread>
#include <tf2_ros/buffer.h>

#include "costmap_layer.h"
#include "footprint.h"
#include "inflation_layer.h"
#include "layer.h"
#include "layered_costmap.h"
#include "map_common.h"
#include "obstacle_layer.h"
#include "static_layer.h"

namespace roborts_costmap {

typedef struct {
  rclcpp::Time time;
  std::string frame_id;
  Eigen::Vector3f position;
  Eigen::Matrix3f rotation;
} RobotPose;

class CostmapInterface {
 public:
  /**
   * @param map_name Name prefix for ROS topics and layered plugin namespaces.
   * @param node ROS 2 node for publishers, timers, and layer subscriptions (must stay alive until destruction).
   * @param tf_buffer TF buffer (typically backed by tf2_ros::TransformListener on the node's executor).
   * @param config_file Path passed to Proto config resolvers (possibly relative within the package share prefix).
   */
  CostmapInterface(std::string map_name, rclcpp::Node::SharedPtr node, tf2_ros::Buffer &tf_buffer,
                   std::string config_file);
  ~CostmapInterface();

  /** @brief Return the ROS 2 node used by this interface (publisher, timers, layer subs). */
  rclcpp::Node::SharedPtr GetNode() const { return node_; }

  void Start();
  void Stop();
  void Resume();
  void UpdateMap();
  void Pause();
  void ResetLayers();

  bool IsCurrent() {
    return layered_costmap_->IsCurrent();
  }

  bool GetRobotPose(geometry_msgs::msg::PoseStamped &global_pose) const;

  Costmap2D *GetCostMap() const {
    return layered_costmap_->GetCostMap();
  }

  std::string GetGlobalFrameID() {
    return global_frame_;
  }

  std::string GetBaseFrameID() {
    return robot_base_frame_;
  }

  CostmapLayers *GetLayeredCostmap() {
    return layered_costmap_;
  }

  geometry_msgs::msg::Polygon GetRobotFootprintPolygon() {
    return ToPolygon(padded_footprint_);
  }

  std::vector<geometry_msgs::msg::Point> GetRobotFootprint() {
    return padded_footprint_;
  }

  std::vector<geometry_msgs::msg::Point> GetUnpaddedRobotFootprint() {
    return unpadded_footprint_;
  }

  void GetOrientedFootprint(std::vector<geometry_msgs::msg::Point> &oriented_footprint) const;

  void SetUnpaddedRobotFootprint(const std::vector<geometry_msgs::msg::Point> &points);

  void SetUnpaddedRobotFootprintPolygon(const geometry_msgs::msg::Polygon &footprint);

  void GetFootprint(std::vector<Eigen::Vector3f> &footprint);

  void GetOrientedFootprint(std::vector<Eigen::Vector3f> &footprint);

  bool GetRobotPose(RobotPose &pose);

  unsigned char *GetCharMap() const;

  geometry_msgs::msg::PoseStamped Pose2GlobalFrame(const geometry_msgs::msg::PoseStamped &pose_msg);

  void ClearCostMap();

  void ClearLayer(CostmapLayer *costmap_layer_ptr, double pose_x, double pose_y);

 protected:
  void LoadParameter();

  std::vector<geometry_msgs::msg::Point> footprint_points_;
  CostmapLayers *layered_costmap_;
  std::string name_, config_file_, config_file_inflation_;
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer &tf_;
  std::string global_frame_, robot_base_frame_;
  double transform_tolerance_, dist_behind_robot_threshold_to_care_obstacles_;
  nav_msgs::msg::OccupancyGrid grid_;
  char *cost_translation_table_ = new char[256];

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

 private:
  void DetectMovement();
  void MapUpdateLoop(double frequency);

  std::vector<geometry_msgs::msg::Point> unpadded_footprint_, padded_footprint_;
  float footprint_padding_;
  bool map_update_thread_shutdown_, stop_updates_, initialized_, stopped_, robot_stopped_, got_footprint_, is_debug_, \
       is_track_unknown_, is_rolling_window_, has_static_layer_, has_obstacle_layer_;
  double map_update_frequency_, map_width_, map_height_, map_origin_x_, map_origin_y_, map_resolution_;
  std::thread *map_update_thread_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_publish_;
  geometry_msgs::msg::PoseStamped old_pose_;
};

} //namespace roborts_costmap
#endif // ROBORTS_COSTMAP_COSTMAP_INTERFACE_H
