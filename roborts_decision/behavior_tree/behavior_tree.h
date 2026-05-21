/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/

#ifndef ROBORTS_DECISION_BEHAVIOR_TREE_H
#define ROBORTS_DECISION_BEHAVIOR_TREE_H

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "behavior_node.h"

namespace roborts_decision {

class BehaviorTree {
 public:
  BehaviorTree(BehaviorNode::Ptr root_node, int cycle_duration,
               rclcpp::Node::SharedPtr spin_node = nullptr)
      : root_node_(root_node),
        cycle_duration_(cycle_duration),
        spin_node_(std::move(spin_node)) {}

  void Run() {
    unsigned int frame = 0;
    while (rclcpp::ok()) {
      if (spin_node_) {
        rclcpp::spin_some(spin_node_);
      }

      auto start_time = std::chrono::steady_clock::now();
      RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Frame : %u", frame);
      root_node_->Run();

      auto end_time = std::chrono::steady_clock::now();
      auto execution_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      auto sleep_time = cycle_duration_ - execution_duration;

      if (sleep_time.count() > 0) {
        std::this_thread::sleep_for(sleep_time);
        RCLCPP_INFO(rclcpp::get_logger("behavior_tree"),
                    "Execution Duration: %ld / %ld ms", execution_duration.count(),
                    cycle_duration_.count());
      } else {
        RCLCPP_WARN(rclcpp::get_logger("behavior_tree"),
                    "The time tick once is %ld beyond the expected time %ld",
                    execution_duration.count(), cycle_duration_.count());
      }

      RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "----------------------------------");
      frame++;
    }
  }

 private:
  BehaviorNode::Ptr root_node_;
  std::chrono::milliseconds cycle_duration_;
  rclcpp::Node::SharedPtr spin_node_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_BEHAVIOR_TREE_H
