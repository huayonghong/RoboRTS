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

#include <iostream>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <roborts_msgs/action/armor_detection.hpp>

using ArmorDetectionAction = roborts_msgs::action::ArmorDetection;
using GoalHandleArmor = rclcpp_action::ClientGoalHandle<ArmorDetectionAction>;

static void SendCommand(const rclcpp_action::Client<ArmorDetectionAction>::SharedPtr &client,
                        int32_t command) {
  if (!client->action_server_is_ready()) {
    return;
  }
  ArmorDetectionAction::Goal goal;
  goal.command = command;
  auto opts = rclcpp_action::Client<ArmorDetectionAction>::SendGoalOptions();
  opts.goal_response_callback = [](std::shared_ptr<GoalHandleArmor> gh) {
    (void)gh;
  };
  opts.result_callback = [](const GoalHandleArmor::WrappedResult & /*result*/) {};
  client->async_send_goal(goal, opts);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("armor_detection_node_test_client");

  auto ac = rclcpp_action::create_client<ArmorDetectionAction>(
      node, "armor_detection_node_action");

  RCLCPP_INFO(node->get_logger(), "Waiting for action server to start.");

  auto spin_thread = std::thread([&]() { rclcpp::spin(node); });

  while (!ac->wait_for_action_server(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      spin_thread.join();
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Still waiting...");
  }

  RCLCPP_INFO(node->get_logger(), "Connected.");
  ArmorDetectionAction::Goal goal;

  char command = '0';

  while (command != '4' && rclcpp::ok()) {
    std::cout << "**************************************************************************************"
              << std::endl;
    std::cout << "*********************************please send a command********************************"
              << std::endl;
    std::cout << "1: start the action" << std::endl
              << "2: pause the action" << std::endl
              << "3: stop  the action" << std::endl
              << "4: exit the program" << std::endl;
    std::cout << "**************************************************************************************"
              << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4') {
      std::cout << "please inpugain!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

    switch (command) {
      case '1':
        goal.command = 1;
        RCLCPP_INFO(node->get_logger(), "Sending start.");
        SendCommand(ac, 1);
        break;
      case '2':
        goal.command = 2;
        RCLCPP_INFO(node->get_logger(), "Sending pause.");
        SendCommand(ac, 2);
        [[fallthrough]];
      case '3':
        goal.command = 3;
        RCLCPP_INFO(node->get_logger(), "Cancelling all goals.");
        ac->async_cancel_all_goals();
        break;
      default:
        break;
    }
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
