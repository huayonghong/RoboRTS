/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_POSITION_COST_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_POSITION_COST_H

#include <vector>
#include <limits>

#include <Eigen/Core>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>

#include "costmap/costmap_interface.h"
#include "local_planner/utility_tool.h"
#include "local_planner/line_iterator.h"
#include "local_planner/robot_footprint_model.h"

namespace roborts_local_planner {

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

class RobotPositionCost {
 public:
  RobotPositionCost(const roborts_costmap::Costmap2D &cost_map);
  ~RobotPositionCost();

  static void CalculateMinAndMaxDistances(const std::vector<Eigen::Vector2d> &footprint,
                                            double &min_dist, double &max_dist) {
    min_dist = std::numeric_limits<double>::max();
    max_dist = 0.0;

    if (footprint.size() <= 2) {
      return;
    }

    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
      double vertex_dist = Distance(0.0, 0.0, footprint[i].coeffRef(0), footprint[i].coeffRef(1));
      double edge_dist = PointToLineDistance(
          0.0, 0.0, footprint[i].coeffRef(0), footprint[i].coeffRef(1), footprint[i + 1].coeffRef(0),
          footprint[i + 1].coeffRef(1));
      min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
      max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    double vertex_dist = Distance(0.0, 0.0, footprint.back().coeffRef(0), footprint.back().coeffRef(1));
    double edge_dist = PointToLineDistance(
        0.0, 0.0, footprint.back().coeffRef(0), footprint.back().coeffRef(1), footprint.front().coeffRef(0),
        footprint.front().coeffRef(1));
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  double FootprintCost(const Eigen::Vector2d &position, const std::vector<Eigen::Vector2d> &footprint);

  double FootprintCost(double x, double y, double theta, const std::vector<Eigen::Vector2d> &footprint_spec,
                       double inscribed_radius = 0.0, double circumscribed_radius = 0.0);

  double LineCost(int x0, int x1, int y0, int y1);

  double PointCost(int x, int y);

 private:
  const roborts_costmap::Costmap2D &costmap_;
};

template <class T>
RobotFootprintModelPtr GetRobotFootprintModel(T config) {
  if (!config.robot_type(0).has_type()) {
    RCLCPP_INFO(rclcpp::get_logger("local_planner"),
                "No robot footprint model specified. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }
  if (config.robot_type(0).type() == 0) {
    RCLCPP_INFO(rclcpp::get_logger("local_planner"), "Footprint model 'point' loaded");
    return boost::make_shared<PointRobotFootprint>();
  }
  if (config.robot_type(0).type() == 1) {
    double radius = 0.0;
    if (!config.robot_type(0).has_radius()) {
      RCLCPP_INFO(rclcpp::get_logger("local_planner"),
                  "the circle radius doesn't exist, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    radius = config.robot_type(0).radius();
    RCLCPP_INFO(rclcpp::get_logger("local_planner"),
                "Footprint model 'circular' (radius: %f m) loaded", radius);
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  if (config.robot_type(0).type() == 3) {
    if (!config.robot_type(0).robot_vertices_size()) {
      RCLCPP_INFO(rclcpp::get_logger("local_planner"),
                  "the line vertices don't exist, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    Eigen::Vector2d line_start, line_end;
    line_start.coeffRef(0) = config.robot_type(0).robot_vertices(0).x();
    line_start.coeffRef(1) = config.robot_type(0).robot_vertices(0).y();

    line_end.coeffRef(0) = config.robot_type(0).robot_vertices(1).x();
    line_end.coeffRef(1) = config.robot_type(0).robot_vertices(1).y();

    if (config.robot_type(0).robot_vertices_size() != 2) {
      RCLCPP_INFO(rclcpp::get_logger("local_planner"),
                  "the vertices' size doesn't equal 2, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }

    RCLCPP_INFO(rclcpp::get_logger("local_planner"), "Footprint model 'line' loaded");
    return boost::make_shared<LineRobotFootprint>(line_start, line_end);
  }
  if (config.robot_type(0).type() == 2) {
    if (!config.robot_type(0).front_offset() || !config.robot_type(0).front_radius() ||
        !config.robot_type(0).rear_offset() || !config.robot_type(0).rear_radius()) {
      RCLCPP_INFO(rclcpp::get_logger("local_planner"),
                  "the param is not enough to combined two circle, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset = config.robot_type(0).front_offset();
    double front_radius = config.robot_type(0).front_radius();
    double rear_offset = config.robot_type(0).rear_offset();
    double rear_radius = config.robot_type(0).rear_radius();

    RCLCPP_INFO(rclcpp::get_logger("local_planner"), "Footprint model 'two circle' loaded");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset,
                                                        rear_radius);
  }
  if (config.robot_type(0).type() == 4) {
    if (!config.robot_type(0).robot_vertices_size()) {
      RCLCPP_INFO(rclcpp::get_logger("local_planner"),
                  "the vertices is null, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    Point2dContainer polygon;
    for (int j = 0; j < config.robot_type(0).robot_vertices_size(); ++j) {
      Eigen::Vector2d temp_point;
      temp_point.coeffRef(0) = config.robot_type(0).robot_vertices(j).x();
      temp_point.coeffRef(1) = config.robot_type(0).robot_vertices(j).y();
      polygon.push_back(temp_point);
    }

    RCLCPP_INFO(rclcpp::get_logger("local_planner"), "Footprint model 'polygon' loaded");
    return boost::make_shared<PolygonRobotFootprint>(polygon);
  }

  RCLCPP_INFO(rclcpp::get_logger("local_planner"),
              "this type doesn't exist, Using point-shaped model");
  return boost::make_shared<PointRobotFootprint>();
}

}  // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_POSITION_COST_H
