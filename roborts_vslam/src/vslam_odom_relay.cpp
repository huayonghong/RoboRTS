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

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * Relay node that converts RTAB-Map's visual odometry output into
 * a standard nav_msgs/Odometry message suitable for robot_localization EKF fusion.
 *
 * RTAB-Map publishes its own TF (map→odom) and odom topic, but when fusing
 * with wheel odometry via EKF, we need a clean Odometry message on a dedicated topic.
 */
class VslamOdomRelay : public rclcpp::Node {
 public:
  VslamOdomRelay() : Node("vslam_odom_relay") {
    this->declare_parameter<std::string>("input_odom_topic", "/rtabmap/odom");
    this->declare_parameter<std::string>("output_odom_topic", "/visual_odom");
    this->declare_parameter<bool>("publish_tf", false);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");

    auto input_topic = this->get_parameter("input_odom_topic").as_string();
    auto output_topic = this->get_parameter("output_odom_topic").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        input_topic, 10,
        std::bind(&VslamOdomRelay::OdomCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(this->get_logger(),
                "VSLAM odom relay: %s -> %s (tf=%s)",
                input_topic.c_str(), output_topic.c_str(),
                publish_tf_ ? "true" : "false");
  }

 private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto out_msg = *msg;
    out_msg.header.frame_id = odom_frame_;
    out_msg.child_frame_id = base_frame_;
    odom_pub_->publish(out_msg);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = out_msg.header;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = out_msg.pose.pose.position.x;
      tf_msg.transform.translation.y = out_msg.pose.pose.position.y;
      tf_msg.transform.translation.z = out_msg.pose.pose.position.z;
      tf_msg.transform.rotation = out_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_;
  std::string odom_frame_;
  std::string base_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VslamOdomRelay>());
  rclcpp::shutdown();
  return 0;
}
