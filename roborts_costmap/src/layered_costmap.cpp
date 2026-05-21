/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include "layered_costmap.h"

namespace roborts_costmap {

CostmapLayers::CostmapLayers(std::string global_frame, bool rolling_window, bool track_unknown) : costmap_(), \
                             global_frame_id_(global_frame), is_rolling_window_(rolling_window), is_initialized_(false), \
                             is_size_locked_(false), file_path_("") {
  if (track_unknown) {
    costmap_.SetDefaultValue(255);
  } else {
    costmap_.SetDefaultValue(0);
  }
}

CostmapLayers::~CostmapLayers() {
  for (auto i = plugins_.size(); i > 0; i--) {
    delete plugins_[i - 1];
  }
  plugins_.clear();
}

bool CostmapLayers::IsCurrent() {
  is_current_ = true;
  for (auto it = plugins_.begin(); it != plugins_.end(); ++it) {
    is_current_ = is_current_ && (*it)->IsCurrent();
  }
  return is_current_;
}

void CostmapLayers::ResizeMap(unsigned int size_x,
                              unsigned int size_y,
                              double resolution,
                              double origin_x,
                              double origin_y,
                              bool size_locked) {
  is_size_locked_ = size_locked;
  costmap_.ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (auto it = plugins_.begin(); it != plugins_.end(); ++it) {
    (*it)->MatchSize();
  }
}

void CostmapLayers::UpdateMap(double robot_x, double robot_y, double robot_yaw) {
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.GetMutex()));
  if (is_rolling_window_) {
    double new_origin_x = robot_x - costmap_.GetSizeXWorld() / 2;
    double new_origin_y = robot_y - costmap_.GetSizeYWorld() / 2;
    costmap_.UpdateOrigin(new_origin_x, new_origin_y);
  }
  if (plugins_.size() == 0) {
    RCLCPP_WARN(rclcpp::get_logger("costmap"), "No Layer");
    return;
  }

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;
  for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->UpdateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(rclcpp::get_logger("costmap"), "Illegal bounds change. The offending layer is %s",
                  (*plugin)->GetName().c_str());
    }
  }
  int x0, xn, y0, yn;
  costmap_.World2MapWithBoundary(minx_, miny_, x0, y0);
  costmap_.World2MapWithBoundary(maxx_, maxy_, xn, yn);
  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.GetSizeXCell()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.GetSizeYCell()), yn + 1);
  if (xn < x0 || yn < y0) {
    return;
  }
  costmap_.ResetPartMap(x0, y0, xn, yn);
  for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
    (*plugin)->UpdateCosts(costmap_, x0, y0, xn, yn);
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;
  is_initialized_ = true;
}

void CostmapLayers::SetFootprint(const std::vector<geometry_msgs::msg::Point> &footprint_spec) {
  footprint_ = footprint_spec;
  CalculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);
  for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
    (*plugin)->OnFootprintChanged();
  }
}

} //namespace roborts_costmap
