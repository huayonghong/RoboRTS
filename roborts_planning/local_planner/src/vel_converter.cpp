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
#include <geometry_msgs/msg/twist.hpp>
#include "roborts_msgs/msg/twist_accel.hpp"

class VelConverter : public rclcpp::Node {
 public:
  VelConverter() : Node("vel_converter") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
    cmd_vel_acc_sub_ = this->create_subscription<roborts_msgs::msg::TwistAccel>(
      "cmd_vel_acc", 1, std::bind(&VelConverter::VelAccCallback, this, std::placeholders::_1));
  }

 private:
  void VelAccCallback(const roborts_msgs::msg::TwistAccel::SharedPtr msg) {
    geometry_msgs::msg::Twist twist;
    twist = msg->twist;
    cmd_vel_pub_->publish(twist);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<roborts_msgs::msg::TwistAccel>::SharedPtr cmd_vel_acc_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelConverter>());
  rclcpp::shutdown();
  return 0;
}
