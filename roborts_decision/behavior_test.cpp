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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/search_behavior.h"

void Command();

char command = '0';

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("behavior_test_node");
  std::string full_path = ament_index_cpp::get_package_share_directory("roborts_decision") +
                          "/config/decision.prototxt";

  auto chassis = std::make_shared<roborts_decision::ChassisExecutor>(node);
  auto blackboard = std::make_shared<roborts_decision::Blackboard>(node, full_path);

  roborts_decision::ChassisExecutor *chassis_executor = chassis.get();
  roborts_decision::Blackboard *bb = blackboard.get();

  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, bb, full_path);
  roborts_decision::ChaseBehavior chase_behavior(chassis_executor, bb, full_path);
  roborts_decision::SearchBehavior search_behavior(chassis_executor, bb, full_path);
  roborts_decision::EscapeBehavior escape_behavior(chassis_executor, bb, full_path);
  roborts_decision::PatrolBehavior patrol_behavior(chassis_executor, bb, full_path);
  roborts_decision::GoalBehavior goal_behavior(chassis_executor, bb);

  auto command_thread = std::thread(Command);
  rclcpp::WallRate rate(10.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    switch (command) {
      case '1':
        back_boot_area_behavior.Run();
        break;
      case '2':
        patrol_behavior.Run();
        break;
      case '3':
        chase_behavior.Run();
        break;
      case '4':
        search_behavior.Run();
        break;
      case '5':
        escape_behavior.Run();
        break;
      case '6':
        goal_behavior.Run();
        break;
      case 27:
        if (command_thread.joinable()) {
          command_thread.join();
        }
        rclcpp::shutdown();
        return 0;
      default:
        break;
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

void Command() {
  while (command != 27) {
    std::cout << "**************************************************************************************"
              << std::endl;
    std::cout << "*********************************please send a command********************************"
              << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************"
              << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' &&
        command != '6' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }
  }
}
