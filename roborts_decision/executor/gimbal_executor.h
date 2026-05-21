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

#ifndef ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#define ROBORTS_DECISION_GIMBAL_EXECUTOR_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <roborts_msgs/msg/gimbal_angle.hpp>
#include <roborts_msgs/msg/gimbal_rate.hpp>

#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {

class GimbalExecutor {
 public:
  enum class ExcutionMode {
    IDLE_MODE,
    ANGLE_MODE,
    RATE_MODE
  };

  explicit GimbalExecutor(rclcpp::Node::SharedPtr node);
  ~GimbalExecutor() = default;

  void Execute(const roborts_msgs::msg::GimbalAngle &gimbal_angle);
  void Execute(const roborts_msgs::msg::GimbalRate &gimbal_rate);

  BehaviorState Update();
  void Cancel();

 private:
  ExcutionMode excution_mode_;
  BehaviorState execution_state_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<roborts_msgs::msg::GimbalRate>::SharedPtr cmd_gimbal_rate_pub_;
  roborts_msgs::msg::GimbalRate zero_gimbal_rate_;

  rclcpp::Publisher<roborts_msgs::msg::GimbalAngle>::SharedPtr cmd_gimbal_angle_pub_;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_GIMBAL_EXECUTOR_H
