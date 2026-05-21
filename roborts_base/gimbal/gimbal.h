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

#ifndef ROBORTS_BASE_GIMBAL_H
#define ROBORTS_BASE_GIMBAL_H
#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"

namespace roborts_base {

/**
 * @brief ROS API for gimbal module
 */
class Gimbal {
 public:
  /**
   * @brief Constructor of gimbal including initialization of sdk and ROS
   * @param handle handler of sdk
   * @param node ROS2 node shared pointer
   */
  Gimbal(std::shared_ptr<roborts_sdk::Handle> handle, rclcpp::Node::SharedPtr node);
  /**
   * @brief Destructor of gimbal
   */
  ~Gimbal();
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
   * @brief Gimbal information callback in sdk
   * @param gimbal_info Gimbal information
   */
  void GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info);
  /**
   * @brief Gimbal angle control callback in ROS
   * @param msg Gimbal angle control command
   */
  void GimbalAngleCtrlCallback(const roborts_msgs::msg::GimbalAngle::SharedPtr msg);

  /**
   * @brief Gimbal mode set service callback in ROS
   * @param req Service request
   * @param res Service response
   */
  void SetGimbalModeService(const std::shared_ptr<roborts_msgs::srv::GimbalMode::Request> req,
                            std::shared_ptr<roborts_msgs::srv::GimbalMode::Response> res);
  /**
   * @brief Control friction wheel service callback in ROS
   * @param req Service request
   * @param res Service response
   */
  void CtrlFricWheelService(const std::shared_ptr<roborts_msgs::srv::FricWhl::Request> req,
                            std::shared_ptr<roborts_msgs::srv::FricWhl::Response> res);
  /**
   * @brief Control shoot service callback in ROS
   * @param req Service request
   * @param res Service response
   */
  void CtrlShootService(const std::shared_ptr<roborts_msgs::srv::ShootCmd::Request> req,
                        std::shared_ptr<roborts_msgs::srv::ShootCmd::Response> res);

  //! sdk handler
  std::shared_ptr<roborts_sdk::Handle> handle_;
  //! ROS2 node
  rclcpp::Node::SharedPtr node_;

  //! sdk version client
  std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                      roborts_sdk::cmd_version_id>> verison_client_;
  //! sdk heartbeat thread
  std::thread heartbeat_thread_;
  //! sdk publisher for heartbeat
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;
  //! sdk publisher for gimbal angle control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_gimbal_angle>>     gimbal_angle_pub_;
  //! sdk publisher for gimbal mode
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::gimbal_mode_e>>        gimbal_mode_pub_;
  //! sdk publisher for friction wheel speed
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_fric_wheel_speed>> fric_wheel_pub_;
  //! sdk publisher for shoot command
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_shoot_info>>       gimbal_shoot_pub_;

  //! ros subscriber for gimbal angle control
  rclcpp::Subscription<roborts_msgs::msg::GimbalAngle>::SharedPtr ros_sub_cmd_gimbal_angle_;
  //! ros service for setting gimbal mode
  rclcpp::Service<roborts_msgs::srv::GimbalMode>::SharedPtr ros_gimbal_mode_srv_;
  //! ros service for controlling friction wheels
  rclcpp::Service<roborts_msgs::srv::FricWhl>::SharedPtr ros_ctrl_fric_wheel_srv_;
  //! ros service for shoot control
  rclcpp::Service<roborts_msgs::srv::ShootCmd>::SharedPtr ros_ctrl_shoot_srv_;
  //! ros gimbal transform relative to base
  geometry_msgs::msg::TransformStamped gimbal_tf_;
  //! ros gimbal tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}
#endif //ROBORTS_BASE_GIMBAL_H
