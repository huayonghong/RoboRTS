/***************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 ***************************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_DATA_CONVERTER_H_
#define ROBORTS_PLANNING_LOCAL_PLANNER_DATA_CONVERTER_H_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2/utils.h>
#include <Eigen/Core>

#include "costmap/costmap_interface.h"

namespace roborts_local_planner {

class DataConverter {
 public:
  static std::pair<Eigen::Vector2d, double> LocalConvertRMData(const roborts_costmap::RobotPose &pose) {
    Eigen::Vector2d position;
    position.coeffRef(0) = pose.position.coeffRef(0);
    position.coeffRef(1) = pose.position.coeffRef(1);
    Eigen::Vector3f euler = pose.rotation.eulerAngles(2, 1, 0);
    return std::make_pair(position, euler(0, 0));
  }

  static std::pair<Eigen::Vector2d, double> LocalConvertGData(const geometry_msgs::msg::Pose &pose) {
    Eigen::Vector2d position;
    position.coeffRef(0) = pose.position.x;
    position.coeffRef(1) = pose.position.y;
    return std::make_pair(position, tf2::getYaw(pose.orientation));
  }

  static Eigen::Vector2d LocalConvertGData(const geometry_msgs::msg::Point &point) {
    Eigen::Vector2d position;
    position.coeffRef(0) = point.x;
    position.coeffRef(1) = point.y;
    return position;
  }

  static std::vector<Eigen::Vector2d> LocalConvertGData(const std::vector<geometry_msgs::msg::Point> &points) {
    std::vector<Eigen::Vector2d> positions;
    for (unsigned int i = 0; i < points.size(); ++i) {
      positions.push_back(LocalConvertGData(points[i]));
    }
    return positions;
  }

  static std::pair<Eigen::Vector2d, double> LocalConvertCData(double x, double y, double theta) {
    Eigen::Vector2d position;
    position.coeffRef(0) = x;
    position.coeffRef(1) = y;
    return std::make_pair(position, theta);
  }
};

}  // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_LOCAL_PLANNER_DATA_CONVERTER_H_
