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

#include <functional>
#include <signal.h>
#include <unistd.h>

#include "armor_detection_node.h"

namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode()
    : Node("armor_detection_node"),
      node_state_(roborts_common::IDLE),
      demensions_(3),
      initialized_(false),
      detected_enemy_(false),
      undetected_count_(0) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  initialized_ = false;
  enemy_info_pub_ = create_publisher<roborts_msgs::msg::GimbalAngle>("cmd_gimbal_angle",
                                                                      rclcpp::QoS(100));
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = roborts_common::IDLE;
  } else {
    RCLCPP_ERROR(get_logger(), "armor_detection_node initalized failed!");
    node_state_ = roborts_common::FAILURE;
  }
  action_server_ = rclcpp_action::create_server<ArmorDetectionAction>(
      shared_from_this(), "armor_detection_node_action",
      std::bind(&ArmorDetectionNode::HandleGoal, this, _1, _2),
      std::bind(&ArmorDetectionNode::HandleCancel, this, _1),
      std::bind(&ArmorDetectionNode::HandleAccepted, this, _1));
}

ErrorInfo ArmorDetectionNode::Init() {
  ArmorDetectionAlgorithms armor_detection_param;

  std::string file_name =
      ament_index_cpp::get_package_share_directory("roborts_detection") +
      "/armor_detection/config/armor_detection.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
  if (!read_state) {
    RCLCPP_ERROR(get_logger(), "Cannot open %s", file_name.c_str());
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }
  gimbal_control_.Init(armor_detection_param.camera_gimbal_transform().offset_x(),
                       armor_detection_param.camera_gimbal_transform().offset_y(),
                       armor_detection_param.camera_gimbal_transform().offset_z(),
                       armor_detection_param.camera_gimbal_transform().offset_pitch(),
                       armor_detection_param.camera_gimbal_transform().offset_yaw(),
                       armor_detection_param.projectile_model_info().init_v(),
                       armor_detection_param.projectile_model_info().init_k());

  std::string selected_algorithm = armor_detection_param.selected_algorithm();
  cv_toolbox_ = std::make_shared<CVToolbox>(armor_detection_param.camera_name());
  armor_detector_ = roborts_common::AlgorithmFactory<ArmorDetectionBase, std::shared_ptr<CVToolbox>>::CreateAlgorithm(
      selected_algorithm, cv_toolbox_);

  undetected_armor_delay_ = armor_detection_param.undetected_armor_delay();
  if (armor_detector_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "Create armor_detector_ pointer failed!");
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }
  return ErrorInfo(ErrorCode::OK);
}

rclcpp_action::GoalResponse ArmorDetectionNode::HandleGoal(const rclcpp_action::GoalUUID &,
                                                             std::shared_ptr<const ArmorDetectionAction::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmorDetectionNode::HandleCancel(const std::shared_ptr<GoalHandleArmor>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmorDetectionNode::HandleAccepted(const std::shared_ptr<GoalHandleArmor> goal_handle) {
  std::thread(std::bind(&ArmorDetectionNode::ExecuteActionGoal, this, goal_handle)).detach();
}

void ArmorDetectionNode::ExecuteActionGoal(std::shared_ptr<GoalHandleArmor> goal_handle) {
  auto feedback = std::make_shared<ArmorDetectionAction::Feedback>();
  auto result = std::make_shared<ArmorDetectionAction::Result>();
  bool undetected_msg_published = false;

  if (!initialized_) {
    feedback->error_code = error_info_.error_code();
    feedback->error_msg = error_info_.error_msg();
    goal_handle->publish_feedback(feedback);
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(), "Initialization Failed, Failed to execute action!");
    return;
  }

  auto goal = goal_handle->get_goal();

  switch (goal->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }

  rclcpp::WallRate rate(25);
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (undetected_count_ != 0) {
        feedback->detected = true;
        feedback->error_code = error_info_.error_code();
        feedback->error_msg = error_info_.error_msg();

        feedback->enemy_pos.header.frame_id = "camera0";
        feedback->enemy_pos.header.stamp = now();

        feedback->enemy_pos.pose.position.x = x_;
        feedback->enemy_pos.pose.position.y = y_;
        feedback->enemy_pos.pose.position.z = z_;
        feedback->enemy_pos.pose.orientation.w = 1;
        goal_handle->publish_feedback(feedback);
        undetected_msg_published = false;
      } else if (!undetected_msg_published) {
        feedback->detected = false;
        feedback->error_code = error_info_.error_code();
        feedback->error_msg = error_info_.error_msg();

        feedback->enemy_pos.header.frame_id = "camera0";
        feedback->enemy_pos.header.stamp = now();

        feedback->enemy_pos.pose.position.x = 0;
        feedback->enemy_pos.pose.position.y = 0;
        feedback->enemy_pos.pose.position.z = 0;
        feedback->enemy_pos.pose.orientation.w = 1;
        goal_handle->publish_feedback(feedback);
        undetected_msg_published = true;
      }
    }
    rate.sleep();
  }
}

void ArmorDetectionNode::ExecuteLoop() {
  undetected_count_ = undetected_armor_delay_;

  while (running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) {
      cv::Point3f target_3d;
      ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, target_3d);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        x_ = target_3d.x;
        y_ = target_3d.y;
        z_ = target_3d.z;
        error_info_ = error_info;
      }

      if (detected_enemy_) {
        float pitch, yaw;
        gimbal_control_.Transform(target_3d, pitch, yaw);

        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        gimbal_angle_.yaw_angle = yaw * 0.7f;
        gimbal_angle_.pitch_angle = pitch;

        std::lock_guard<std::mutex> guard(mutex_);
        undetected_count_ = undetected_armor_delay_;
        PublishMsgs();
      } else if (undetected_count_ != 0) {
        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        gimbal_angle_.yaw_angle = 0;
        gimbal_angle_.pitch_angle = 0;

        undetected_count_--;
        PublishMsgs();
      }
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}

void ArmorDetectionNode::PublishMsgs() {
  enemy_info_pub_->publish(gimbal_angle_);
}

void ArmorDetectionNode::StartThread() {
  RCLCPP_INFO(get_logger(), "Armor detection node started!");
  running_ = true;
  armor_detector_->SetThreadState(true);
  if (node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() {
  RCLCPP_INFO(get_logger(), "Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  armor_detector_->SetThreadState(false);
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

ArmorDetectionNode::~ArmorDetectionNode() {
  StopThread();
}

} //namespace roborts_detection

void SignalHandler(int signal) {
  (void)signal;
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<roborts_detection::ArmorDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
