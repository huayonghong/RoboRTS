/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

#include <iostream>

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

class PatrolBehavior {
 public:
  PatrolBehavior(ChassisExecutor *&chassis_executor, Blackboard *&blackboard,
                 const std::string &proto_file_path)
      : chassis_executor_(chassis_executor), blackboard_(blackboard) {
    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      RCLCPP_ERROR(rclcpp::get_logger("patrol_behavior"), "%s can't open file", __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = Update();

    std::cout << "state: " << static_cast<int>(executor_state) << std::endl;

    if (executor_state != BehaviorState::RUNNING) {
      if (patrol_goals_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("patrol_behavior"), "patrol goal is empty");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
    }
  }

  void Cancel() { chassis_executor_->Cancel(); }

  BehaviorState Update() { return chassis_executor_->Update(); }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = static_cast<unsigned int>(decision_config.point().size());
    patrol_goals_.resize(point_size_);
    for (int i = 0; static_cast<unsigned int>(i) != point_size_; i++) {
      patrol_goals_[static_cast<size_t>(i)].header.frame_id = "map";
      patrol_goals_[static_cast<size_t>(i)].pose.position.x =
          decision_config.point(i).x();
      patrol_goals_[static_cast<size_t>(i)].pose.position.y =
          decision_config.point(i).y();
      patrol_goals_[static_cast<size_t>(i)].pose.position.z =
          decision_config.point(i).z();

      patrol_goals_[static_cast<size_t>(i)].pose.orientation =
          QuaternionFromRollPitchYaw(decision_config.point(i).roll(),
                                      decision_config.point(i).pitch(),
                                      decision_config.point(i).yaw());
    }

    return true;
  }

  ~PatrolBehavior() = default;

 private:
  ChassisExecutor *const chassis_executor_;
  Blackboard *const blackboard_;

  std::vector<geometry_msgs::msg::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_PATROL_BEHAVIOR_H
