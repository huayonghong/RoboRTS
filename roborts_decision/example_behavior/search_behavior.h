/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_SEARCH_BEHAVIOR_H
#define ROBORTS_DECISION_SEARCH_BEHAVIOR_H

#include <cmath>

#include "io/io.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../executor/chassis_executor.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"
#include "msg_helpers.hpp"

namespace roborts_decision {

class SearchBehavior {
 public:
  SearchBehavior(ChassisExecutor *&chassis_executor, Blackboard *&blackboard,
                 const std::string &proto_file_path)
      : chassis_executor_(chassis_executor), blackboard_(blackboard) {
    last_position_.header.frame_id = "map";
    last_position_.pose.orientation.x = 0;
    last_position_.pose.orientation.y = 0;
    last_position_.pose.orientation.z = 0;
    last_position_.pose.orientation.w = 1;

    last_position_.pose.position.x = 0;
    last_position_.pose.position.y = 0;
    last_position_.pose.position.z = 0;

    search_index_ = 0;
    search_count_ = 0;

    if (!LoadParam(proto_file_path)) {
      RCLCPP_ERROR(rclcpp::get_logger("search_behavior"), "%s can't open file", __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = Update();

    double yaw = 0;
    double x_diff = 0;
    double y_diff = 0;

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      if (search_count_ == 5) {
        x_diff = last_position_.pose.position.x - robot_map_pose.pose.position.x;
        y_diff = last_position_.pose.position.y - robot_map_pose.pose.position.y;

        auto last_x = last_position_.pose.position.x;
        auto last_y = last_position_.pose.position.y;

        if (last_x < 4.2 && last_y < 2.75) {
          search_region_ = search_region_1_;

        } else if (last_x > 4.2 && last_y < 2.75) {
          search_region_ = search_region_2_;

        } else if (last_x < 4.2 && last_y > 2.75) {
          search_region_ = search_region_3_;

        } else {
          search_region_ = search_region_4_;
        }

        double search_min_dist = 99999;
        for (unsigned int i = 0; i < search_region_.size(); ++i) {
          auto dist_sq = std::pow(search_region_[i].pose.position.x - last_x, 2) +
                         std::pow(search_region_[i].pose.position.y - last_y, 2);

          if (dist_sq < search_min_dist) {
            search_min_dist = dist_sq;
            search_index_ = i;
          }
        }

        yaw = std::atan2(y_diff, x_diff);

        auto orientation = QuaternionFromYaw(yaw);

        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = blackboard_->Now();
        goal.pose.position = last_position_.pose.position;
        goal.pose.orientation = orientation;
        chassis_executor_->Execute(goal);
        search_count_--;

      } else if (search_count_ > 0) {
        auto search_goal = search_region_[(search_index_++)];
        chassis_executor_->Execute(search_goal);
        search_index_ = static_cast<unsigned int>(search_index_ % search_region_.size());
        search_count_--;
      }
    }
  }

  void Cancel() { chassis_executor_->Cancel(); }

  BehaviorState Update() { return chassis_executor_->Update(); }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    for (int i = 0; i != decision_config.search_region_1().size(); i++) {
      geometry_msgs::msg::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_1(i).x();
      search_point.pose.position.y = decision_config.search_region_1(i).y();
      search_point.pose.position.z = decision_config.search_region_1(i).z();

      search_point.pose.orientation =
          QuaternionFromRollPitchYaw(decision_config.search_region_1(i).roll(),
                                      decision_config.search_region_1(i).pitch(),
                                      decision_config.search_region_1(i).yaw());
      search_region_1_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_2().size(); i++) {
      geometry_msgs::msg::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_2(i).x();
      search_point.pose.position.y = decision_config.search_region_2(i).y();
      search_point.pose.position.z = decision_config.search_region_2(i).z();

      search_point.pose.orientation =
          QuaternionFromRollPitchYaw(decision_config.search_region_2(i).roll(),
                                      decision_config.search_region_2(i).pitch(),
                                      decision_config.search_region_2(i).yaw());
      search_region_2_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_3().size(); i++) {
      geometry_msgs::msg::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_3(i).x();
      search_point.pose.position.y = decision_config.search_region_3(i).y();
      search_point.pose.position.z = decision_config.search_region_3(i).z();

      search_point.pose.orientation =
          QuaternionFromRollPitchYaw(decision_config.search_region_3(i).roll(),
                                      decision_config.search_region_3(i).pitch(),
                                      decision_config.search_region_3(i).yaw());
      search_region_3_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_4().size(); i++) {
      geometry_msgs::msg::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_4(i).x();
      search_point.pose.position.y = decision_config.search_region_4(i).y();
      search_point.pose.position.z = decision_config.search_region_4(i).z();

      search_point.pose.orientation =
          QuaternionFromRollPitchYaw(decision_config.search_region_4(i).roll(),
                                      decision_config.search_region_4(i).pitch(),
                                      decision_config.search_region_4(i).yaw());
      search_region_4_.push_back(search_point);
    }
    return true;
  }

  void SetLastPosition(const geometry_msgs::msg::PoseStamped &last_position) {
    last_position_ = last_position;
    search_count_ = 5;
  }

  ~SearchBehavior() = default;

 private:
  ChassisExecutor *const chassis_executor_;
  Blackboard *const blackboard_;

  geometry_msgs::msg::PoseStamped last_position_;

  std::vector<geometry_msgs::msg::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::msg::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::msg::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::msg::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::msg::PoseStamped> search_region_;

  unsigned int search_count_;
  unsigned int search_index_;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_SEARCH_BEHAVIOR_H
