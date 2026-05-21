/***************************************************************************
 * Author: Christoph Rösmann
 ***************************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_OPTIMAL_BASE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_OPTIMAL_BASE_H

#include <boost/shared_ptr.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "local_planner/data_base.h"
#include "state/error_code.h"
#include "local_planner/robot_position_cost.h"

namespace roborts_local_planner {

class OptimalBase {
 public:
  OptimalBase() {}

  virtual bool Optimal(std::vector<DataBase> &initial_plan,
                       const geometry_msgs::msg::Twist *start_vel = NULL,
                       bool free_goal_vel = false, bool micro_control = false) = 0;

  virtual bool Optimal(const DataBase &start, const DataBase &goal,
                       const geometry_msgs::msg::Twist *start_vel = NULL,
                       bool free_goal_vel = false, bool micro_control = false) = 0;

  virtual bool GetVelocity(roborts_common::ErrorInfo &error_info, double &vx, double &vy, double &omega,
                           double &acc_x, double &acc_y, double &acc_omega) const = 0;

  virtual void ClearPlanner() = 0;

  virtual void Visualize() {}

  virtual bool IsTrajectoryFeasible(roborts_common::ErrorInfo &error_info, RobotPositionCost *position_cost,
                                    const std::vector<Eigen::Vector2d> &footprint_spec,
                                    double inscribed_radius = 0.0, double circumscribed_radius = 0.0,
                                    int look_ahead_idx = -1) = 0;

  virtual bool IsHorizonReductionAppropriate(const std::vector<DataBase> &initial_plan) const {
    return false;
  }

  virtual void ComputeCurrentCost(std::vector<double> &cost, double obst_cost_scale = 1.0,
                                  bool alternative_time_cost = false) {}

  virtual ~OptimalBase() {}
};

typedef boost::shared_ptr<OptimalBase> OptimalBasePtr;

}  // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_LOCAL_PLANNER_OPTIMAL_BASE_H
