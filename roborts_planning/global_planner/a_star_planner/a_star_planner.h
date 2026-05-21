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

#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H

#include <limits>
#include <queue>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logging.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "proto/a_star_planner_config.pb.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "costmap/costmap_interface.h"

#include "../global_planner_base.h"

namespace roborts_global_planner{

/**
 * @brief Global planner alogorithm class for A star under the representation of costmap
 */
class AStarPlanner : public GlobalPlannerBase {

 public:
  AStarPlanner(CostmapPtr costmap_ptr);
  virtual ~AStarPlanner();

  roborts_common::ErrorInfo Plan(const geometry_msgs::msg::PoseStamped &start,
                               const geometry_msgs::msg::PoseStamped &goal,
                               std::vector<geometry_msgs::msg::PoseStamped> &path) override;

 private:
  enum SearchState {
    NOT_HANDLED,
    OPEN,
    CLOSED
  };

  roborts_common::ErrorInfo SearchPath(const int &start_index,
                                     const int &goal_index,
                                     std::vector<geometry_msgs::msg::PoseStamped> &path);

  roborts_common::ErrorInfo GetMoveCost(const int &current_index,
                                      const int &neighbor_index,
                                      int &move_cost) const;

  void GetManhattanDistance(const int &index1,
                          const int &index2,
                          int &manhattan_distance) const;

  void GetNineNeighbors(const int &current_index,
                        std::vector<int> &neighbors_index) const;

  struct Compare {
    bool operator()(const int &index1, const int &index2) {
      return AStarPlanner::f_score_.at(index1) > AStarPlanner::f_score_.at(index2);
    }
  };

  float heuristic_factor_;
  unsigned int inaccessible_cost_;
  unsigned int goal_search_tolerance_;
  unsigned int gridmap_height_;
  unsigned int gridmap_width_;
  unsigned char *cost_;
  static std::vector<int> f_score_;
  std::vector<int> g_score_;
  std::vector<int> parent_;
  std::vector<AStarPlanner::SearchState> state_;

};

std::vector<int> AStarPlanner::f_score_;
roborts_common::REGISTER_ALGORITHM(GlobalPlannerBase,
                                 "a_star_planner",
                                 AStarPlanner,
                                 std::shared_ptr<roborts_costmap::CostmapInterface>);

} //namespace roborts_global_planner

#endif // ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
