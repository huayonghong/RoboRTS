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

#ifndef ROBORTS_BASE_REFEREE_SYSTEM_H
#define ROBORTS_BASE_REFEREE_SYSTEM_H

#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"

namespace roborts_base {

/**
 * @brief ROS API for referee system module
 */
class RefereeSystem {
 public:
  /**
   * @brief Constructor of referee system including initialization of sdk and ROS
   * @param handle handler of sdk
   * @param node ROS2 node shared pointer
   */
  RefereeSystem(std::shared_ptr<roborts_sdk::Handle> handle, rclcpp::Node::SharedPtr node);
  /**
   * @brief Destructor of referee system
   */
  ~RefereeSystem() = default;
 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();
  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();

  /**
   * @brief Game state callback in sdk
   * @param raw_game_status Raw game status from referee
   */
  void GameStateCallback(const std::shared_ptr<roborts_sdk::cmd_game_state> raw_game_status);
  /**
   * @brief Game result callback in sdk
   * @param raw_game_result Raw game result from referee
   */
  void GameResultCallback(const std::shared_ptr<roborts_sdk::cmd_game_result> raw_game_result);
  /**
   * @brief Game survivor callback in sdk
   * @param raw_game_survivor Raw survivor state from referee
   */
  void GameSurvivorCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_survivors> raw_game_survivor);
  /**
   * @brief Battlefield event (bonus status) callback in sdk
   * @param raw_game_event Raw event data from referee
   */
  void GameEventCallback(const std::shared_ptr<roborts_sdk::cmd_event_data> raw_game_event);
  /**
   * @brief Supplier status callback in sdk
   * @param raw_supplier_status Raw supplier action from referee
   */
  void SupplierStatusCallback(const std::shared_ptr<roborts_sdk::cmd_supply_projectile_action> raw_supplier_status);
  /**
   * @brief Robot status callback in sdk
   * @param raw_robot_status Raw robot state from referee
   */
  void RobotStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> raw_robot_status);
  /**
   * @brief Robot heat and power callback in sdk
   * @param raw_robot_heat Raw power / heat data from referee
   */
  void RobotHeatCallback(const std::shared_ptr<roborts_sdk::cmd_power_heat_data> raw_robot_heat);
  /**
   * @brief Robot bonus callback in sdk
   * @param raw_robot_bonus Raw buff data from referee
   */
  void RobotBonusCallback(const std::shared_ptr<roborts_sdk::cmd_buff_musk> raw_robot_bonus);
  /**
   * @brief Robot damage callback in sdk
   * @param raw_robot_damage Raw hurt data from referee
   */
  void RobotDamageCallback(const std::shared_ptr<roborts_sdk::cmd_robot_hurt> raw_robot_damage);
  /**
   * @brief Robot shoot callback in sdk
   * @param raw_robot_shoot Raw shoot data from referee
   */
  void RobotShootCallback(const std::shared_ptr<roborts_sdk::cmd_shoot_data> raw_robot_shoot);
  /**
   * @brief Projectile supply request callback in ROS
   * @param projectile_supply Projectile supply request message
   */
  void ProjectileSupplyCallback(const roborts_msgs::msg::ProjectileSupply::SharedPtr projectile_supply);

  //! sdk handler
  std::shared_ptr<roborts_sdk::Handle> handle_;
  //! ROS2 node
  rclcpp::Node::SharedPtr node_;

  //! sdk publisher for projectile supply booking
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_supply_projectile_booking>> projectile_supply_pub_;

  //! ros subscriber for projectile supply request
  rclcpp::Subscription<roborts_msgs::msg::ProjectileSupply>::SharedPtr ros_sub_projectile_supply_;

  //! ros publisher for game status
  rclcpp::Publisher<roborts_msgs::msg::GameStatus>::SharedPtr ros_game_status_pub_;
  //! ros publisher for game result
  rclcpp::Publisher<roborts_msgs::msg::GameResult>::SharedPtr ros_game_result_pub_;
  //! ros publisher for game survivor state
  rclcpp::Publisher<roborts_msgs::msg::GameSurvivor>::SharedPtr ros_game_survival_pub_;
  //! ros publisher for field bonus status
  rclcpp::Publisher<roborts_msgs::msg::BonusStatus>::SharedPtr ros_bonus_status_pub_;
  //! ros publisher for supplier status
  rclcpp::Publisher<roborts_msgs::msg::SupplierStatus>::SharedPtr ros_supplier_status_pub_;
  //! ros publisher for robot status
  rclcpp::Publisher<roborts_msgs::msg::RobotStatus>::SharedPtr ros_robot_status_pub_;
  //! ros publisher for robot heat
  rclcpp::Publisher<roborts_msgs::msg::RobotHeat>::SharedPtr ros_robot_heat_pub_;
  //! ros publisher for robot bonus
  rclcpp::Publisher<roborts_msgs::msg::RobotBonus>::SharedPtr ros_robot_bonus_pub_;
  //! ros publisher for robot damage
  rclcpp::Publisher<roborts_msgs::msg::RobotDamage>::SharedPtr ros_robot_damage_pub_;
  //! ros publisher for robot shoot
  rclcpp::Publisher<roborts_msgs::msg::RobotShoot>::SharedPtr ros_robot_shoot_pub_;

  //! cached robot id from referee (0xFF if unknown)
  uint8_t robot_id_ = 0xFF;
};
}
#endif //ROBORTS_BASE_REFEREE_SYSTEM_H
