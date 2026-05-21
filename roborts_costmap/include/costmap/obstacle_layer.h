#ifndef ROBORTS_COSTMAP_OBSTACLE_LAYER_H
#define ROBORTS_COSTMAP_OBSTACLE_LAYER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "costmap_layer.h"
#include "footprint.h"
#include "layered_costmap.h"
#include "map_common.h"
#include "observation_buffer.h"

namespace roborts_costmap {

class ObstacleLayer : public CostmapLayer {
 public:
  ObstacleLayer() {
    costmap_ = nullptr;
  }

  virtual ~ObstacleLayer() = default;
  virtual void OnInitialize();
  virtual void Activate();
  virtual void Deactivate();
  virtual void Reset();
  virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                            double *min_y, double *max_x, double *max_y) override;

  void LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message,
                         const std::shared_ptr<ObservationBuffer> &buffer);

  void LaserScanValidInfoCallback(const sensor_msgs::msg::LaserScan::SharedPtr raw_message,
                                  const std::shared_ptr<ObservationBuffer> &buffer);

 protected:
  bool GetMarkingObservations(std::vector<Observation> &marking_observations) const;
  bool GetClearingObservations(std::vector<Observation> &clearing_observations) const;
  virtual void RaytraceFreespace(const Observation &clearing_observation, double *min_x, double *min_y,
                                 double *max_x, double *max_y);
  void UpdateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double *min_x,
                            double *min_y, double *max_x, double *max_y);
  void UpdateFootprint(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                       double *max_x, double *max_y);

  bool footprint_clearing_enabled_, rolling_window_;
  int combination_method_;
  std::string global_frame_;
  double max_obstacle_height_;
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  laser_geometry::LaserProjection projector_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> observation_subscribers_;
  std::vector<std::shared_ptr<ObservationBuffer> > observation_buffers_;
  std::vector<std::shared_ptr<ObservationBuffer> > marking_buffers_;
  std::vector<std::shared_ptr<ObservationBuffer> > clearing_buffers_;

  std::vector<Observation> static_clearing_observations_, static_marking_observations_;
  std::chrono::system_clock::time_point reset_time_;
};

} //namespace roborts_costmap

#endif //ROBORTS_COSTMAP_OBSTACLE_LAYER_H
