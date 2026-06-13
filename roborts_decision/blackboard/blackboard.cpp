/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/

#include "blackboard.h"

#include <functional>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "io/io.h"

namespace roborts_decision {

Blackboard::Blackboard(rclcpp::Node::SharedPtr node, const std::string &proto_file_path)
    : node_(std::move(node)), enemy_detected_(false), simulate_(false) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  std::string map_path =
      ament_index_cpp::get_package_share_directory("roborts_costmap") +
      "/config/costmap_parameter_config_for_decision.prototxt";
  costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", node_, *tf_buffer_, map_path);
  charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
  costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

  enemy_pose_.header.frame_id = "map";
  enemy_pose_.pose.orientation.w = 1.0;

  enemy_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        GoalCallback(msg);
      });

  DecisionConfig decision_config;
  roborts_common::ReadProtoFromTextFile(proto_file_path.c_str(), &decision_config);
  simulate_ = decision_config.simulate();

  if (!simulate_) {
    armor_detection_client_ =
        rclcpp_action::create_client<roborts_msgs::action::ArmorDetection>(node_,
                                                                           "armor_detection_node_action");

    while (!armor_detection_client_->wait_for_action_server(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for armor_detection action...");
      rclcpp::spin_some(node_);
    }
    RCLCPP_INFO(node_->get_logger(), "Armor detection module has been connected!");
    roborts_msgs::action::ArmorDetection::Goal armor_goal;
    armor_goal.command = 1;
    auto opts = rclcpp_action::Client<roborts_msgs::action::ArmorDetection>::SendGoalOptions();
    opts.feedback_callback =
        std::bind(&Blackboard::ArmorDetectionFeedbackCb, this, std::placeholders::_1,
                  std::placeholders::_2);
    armor_detection_client_->async_send_goal(armor_goal, opts);
  }
}

Blackboard::~Blackboard() = default;

void Blackboard::ArmorDetectionFeedbackCb(
    ArmorGoalHandle::SharedPtr /*goal_handle_unused*/,
    const std::shared_ptr<const roborts_msgs::action::ArmorDetection::Feedback> feedback) {
  if (feedback->detected) {
    enemy_detected_ = true;
    RCLCPP_INFO(node_->get_logger(), "Find Enemy!");

    geometry_msgs::msg::PoseStamped camera_pose_msg = feedback->enemy_pos;
    double yaw = std::atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    camera_pose_msg.pose.orientation = tf2::toMsg(quaternion);

    try {
      geometry_msgs::msg::PoseStamped global_pose_msg;
      camera_pose_msg.header.stamp = node_->now();
      tf_buffer_->transform(camera_pose_msg, global_pose_msg, "map", tf2::durationFromSec(0.05));

      if (GetDistance(global_pose_msg, enemy_pose_) > 0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2) {
        enemy_pose_ = global_pose_msg;
      }
    } catch (const tf2::TransformException &) {
      RCLCPP_ERROR(node_->get_logger(), "tf error when transform enemy pose from camera to map");
    }
  } else {
    enemy_detected_ = false;
  }
}

bool Blackboard::IsEnemyDetected() const {
  RCLCPP_INFO(node_->get_logger(), "%s: %d", __FUNCTION__, static_cast<int>(enemy_detected_));
  return enemy_detected_;
}

void Blackboard::GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
  new_goal_ = true;
  goal_ = *goal;
}

bool Blackboard::IsNewGoal() {
  if (new_goal_) {
    new_goal_ = false;
    return true;
  }
  return false;
}

double Blackboard::GetDistance(const geometry_msgs::msg::PoseStamped &pose1,
                               const geometry_msgs::msg::PoseStamped &pose2) const {
  const auto &point1 = pose1.pose.position;
  const auto &point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double Blackboard::GetAngle(const geometry_msgs::msg::PoseStamped &pose1,
                            const geometry_msgs::msg::PoseStamped &pose2) const {
  tf2::Quaternion rot1;
  tf2::Quaternion rot2;
  tf2::fromMsg(pose1.pose.orientation, rot1);
  tf2::fromMsg(pose2.pose.orientation, rot2);
  return rot1.angleShortestPath(rot2);
}

geometry_msgs::msg::PoseStamped Blackboard::GetRobotMapPose() {
  if (!costmap_ptr_->GetRobotPose(robot_map_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "Transform Error looking up robot pose.");
  }
  return robot_map_pose_;
}

} // namespace roborts_decision
