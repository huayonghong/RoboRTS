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

#include "gimbal_executor.h"

namespace roborts_decision {

GimbalExecutor::GimbalExecutor(rclcpp::Node::SharedPtr node)
    : excution_mode_(ExcutionMode::IDLE_MODE),
      execution_state_(BehaviorState::IDLE),
      node_(std::move(node)) {
  cmd_gimbal_angle_pub_ =
      node_->create_publisher<roborts_msgs::msg::GimbalAngle>("cmd_gimbal_angle", rclcpp::QoS(1));
  cmd_gimbal_rate_pub_ =
      node_->create_publisher<roborts_msgs::msg::GimbalRate>("cmd_gimbal_rate", rclcpp::QoS(1));
}

void GimbalExecutor::Execute(const roborts_msgs::msg::GimbalAngle &gimbal_angle) {
  excution_mode_ = ExcutionMode::ANGLE_MODE;
  cmd_gimbal_angle_pub_->publish(gimbal_angle);
}

void GimbalExecutor::Execute(const roborts_msgs::msg::GimbalRate &gimbal_rate) {
  excution_mode_ = ExcutionMode::RATE_MODE;
  cmd_gimbal_rate_pub_->publish(gimbal_rate);
}

BehaviorState GimbalExecutor::Update() {
  switch (excution_mode_) {
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::ANGLE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::RATE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel() {
  switch (excution_mode_) {
    case ExcutionMode::IDLE_MODE:
      RCLCPP_WARN(node_->get_logger(), "Nothing to be canceled.");
      break;

    case ExcutionMode::ANGLE_MODE:
      cmd_gimbal_rate_pub_->publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::RATE_MODE:
      cmd_gimbal_rate_pub_->publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Wrong Execution Mode");
  }
}

} // namespace roborts_decision
