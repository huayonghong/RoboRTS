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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <roborts_msgs/action/armor_detection.hpp>
#include <roborts_msgs/msg/gimbal_angle.hpp>

#include "alg_factory/algorithm_factory.h"
#include "io/io.h"
#include "state/node_state.h"

#include "../util/cv_toolbox.h"

#include "armor_detection_base.h"
#include "proto/armor_detection.pb.h"
#include "armor_detection_algorithms.h"
#include "gimbal_control.h"

namespace roborts_detection {

using roborts_common::NodeState;
using roborts_common::ErrorInfo;
using ArmorDetectionAction = roborts_msgs::action::ArmorDetection;
using GoalHandleArmor = rclcpp_action::ServerGoalHandle<ArmorDetectionAction>;

class ArmorDetectionNode : public rclcpp::Node {
 public:
  ArmorDetectionNode();
  ErrorInfo Init();
  void ExecuteActionGoal(std::shared_ptr<GoalHandleArmor> goal_handle);
  void StartThread();
  void PauseThread();
  void StopThread();
  void ExecuteLoop();
  void PublishMsgs();
  ~ArmorDetectionNode();

 private:
  rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const ArmorDetectionAction::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleArmor> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandleArmor> goal_handle);

  std::shared_ptr<ArmorDetectionBase> armor_detector_;
  std::thread armor_detection_thread_;
  unsigned int max_rotating_fps_;
  unsigned int min_rotating_detected_count_;
  unsigned int undetected_armor_delay_;

  NodeState node_state_;
  ErrorInfo error_info_;
  bool initialized_;
  bool running_;
  std::mutex mutex_;
  std::condition_variable condition_var_;
  unsigned int undetected_count_;

  double x_;
  double y_;
  double z_;
  bool detected_enemy_;
  unsigned long demensions_;

  rclcpp::Publisher<roborts_msgs::msg::GimbalAngle>::SharedPtr enemy_info_pub_;

  std::shared_ptr<CVToolbox> cv_toolbox_;
  rclcpp_action::Server<ArmorDetectionAction>::SharedPtr action_server_;

  roborts_msgs::msg::GimbalAngle gimbal_angle_;

  GimbalContrl gimbal_control_;
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
