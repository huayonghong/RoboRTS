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

#ifndef ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H
#define ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "log.h"
#include "localization_config.h"
#include "amcl/amcl.h"
#include "localization_math.h"
#include "types.h"

#define THREAD_NUM 4

namespace roborts_localization {

class LocalizationNode {
 public:
  explicit LocalizationNode(rclcpp::Node::SharedPtr node);

  /**
   * @brief Localization initialization
   * @return Returns true if initialize success
   */
  bool Init();

  void LaserScanCallback(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser_scan_msg);

  void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
                               &init_pose_msg);

  void PublishVisualize();

  bool PublishTf();

 private:
  bool SpinUntil(const std::function<bool()> &predicate);
  bool GetPoseFromTf(const std::string &target_frame,
                     const std::string &source_frame,
                     const rclcpp::Time &timestamp,
                     Vec3d &pose);

  bool WaitForStaticMap();
  bool WaitForLaserPose();

  static bool FramesMatch(std::string a, std::string b);

  void TransformLaserscanToBaseFrame(
      double &angle_min,
      double &angle_increment,
      const sensor_msgs::msg::LaserScan &laser_scan_msg);

 private:
  std::mutex mutex_;
  std::unique_ptr<Amcl> amcl_ptr_;
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
  std::unique_ptr<
      tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
      laser_scan_filter_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr distance_map_pub_;

  std::string odom_frame_;
  std::string global_frame_;
  std::string base_frame_;
  std::string laser_topic_;
  std::string map_topic_;

  Vec3d init_pose_;
  Vec3d init_cov_;
  rclcpp::Duration transform_tolerance_{std::chrono::nanoseconds{0}};
  bool publish_visualize_;

  bool initialized_ = false;
  bool map_init_ = false;
  bool laser_init_ = false;
  bool latest_tf_valid_ = false;
  bool sent_first_transform_ = false;
  bool publish_first_distance_map_ = false;

  HypPose hyp_pose_;
  geometry_msgs::msg::PoseArray particlecloud_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  rclcpp::Time last_laser_msg_timestamp_;

  tf2::Transform latest_tf_;
};

}

#endif

