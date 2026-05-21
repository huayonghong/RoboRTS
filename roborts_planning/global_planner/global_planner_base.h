/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_BASE_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_BASE_H

#include "state/error_code.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "costmap/costmap_interface.h"

namespace roborts_global_planner{

class GlobalPlannerBase {
 public:
  typedef std::shared_ptr<roborts_costmap::CostmapInterface> CostmapPtr;

  GlobalPlannerBase(CostmapPtr costmap_ptr)
      : costmap_ptr_(costmap_ptr) {
  };
  virtual ~GlobalPlannerBase() = default;

  virtual roborts_common::ErrorInfo Plan(const geometry_msgs::msg::PoseStamped &start,
                                       const geometry_msgs::msg::PoseStamped &goal,
                                       std::vector<geometry_msgs::msg::PoseStamped> &path) = 0;

 protected:
  CostmapPtr costmap_ptr_;
};

} //namespace roborts_global_planner

#endif // ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_BASE_H
