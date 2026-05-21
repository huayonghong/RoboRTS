/***************************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 ***************************************************************************/

#ifndef ROBORTS_PLANNING_BASE_LOCAL_PLANNER_H
#define ROBORTS_PLANNING_BASE_LOCAL_PLANNER_H

#include <functional>

#include <boost/shared_ptr.hpp>
#include <tf2_ros/buffer.h>

#include "state/error_code.h"
#include "costmap/costmap_interface.h"
#include "local_planner/local_visualization.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <roborts_msgs/msg/twist_accel.hpp>

namespace roborts_local_planner {

typedef std::function<void(const roborts_common::ErrorInfo &)> ErrorInfoCallback;

class LocalPlannerBase {
 public:
  virtual roborts_common::ErrorInfo ComputeVelocityCommands(roborts_msgs::msg::TwistAccel &cmd_vel) = 0;

  virtual bool IsGoalReached() = 0;

  virtual roborts_common::ErrorInfo Initialize(std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
                                               std::shared_ptr<tf2_ros::Buffer> tf,
                                               LocalVisualizationPtr visual) = 0;

  virtual bool SetPlan(const nav_msgs::msg::Path &plan,
                       const geometry_msgs::msg::PoseStamped &goal) = 0;

  virtual void RegisterErrorCallBack(ErrorInfoCallback error_callback) = 0;

  virtual ~LocalPlannerBase() {}

 protected:
  LocalPlannerBase() {}
};

typedef boost::shared_ptr<LocalPlannerBase> LocalPlannerPtr;

}  // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_BASE_LOCAL_PLANNER_H
