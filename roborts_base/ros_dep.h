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
#ifndef ROBORTS_SDK_PROTOCOL_DEFINE_ROS_H
#define ROBORTS_SDK_PROTOCOL_DEFINE_ROS_H
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "roborts_msgs/msg/twist_accel.hpp"
#include "roborts_msgs/msg/gimbal_angle.hpp"
#include "roborts_msgs/msg/gimbal_rate.hpp"
#include "roborts_msgs/srv/gimbal_mode.hpp"
#include "roborts_msgs/srv/shoot_cmd.hpp"
#include "roborts_msgs/srv/fric_whl.hpp"

#include "roborts_msgs/msg/bonus_status.hpp"
#include "roborts_msgs/msg/game_result.hpp"
#include "roborts_msgs/msg/game_status.hpp"
#include "roborts_msgs/msg/game_survivor.hpp"
#include "roborts_msgs/msg/projectile_supply.hpp"
#include "roborts_msgs/msg/robot_bonus.hpp"
#include "roborts_msgs/msg/robot_damage.hpp"
#include "roborts_msgs/msg/robot_heat.hpp"
#include "roborts_msgs/msg/robot_shoot.hpp"
#include "roborts_msgs/msg/robot_status.hpp"
#include "roborts_msgs/msg/supplier_status.hpp"

#endif //ROBORTS_SDK_PROTOCOL_DEFINE_ROS_H
