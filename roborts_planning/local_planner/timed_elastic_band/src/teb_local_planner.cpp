/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Author: Christoph Rösmann (TU Dortmund)
 *********************************************************************/

#include "timed_elastic_band/teb_local_planner.h"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <roborts_msgs/msg/twist_accel.hpp>

namespace roborts_local_planner {

namespace {
rclcpp::Logger kLocalPlannerLogger() { return rclcpp::get_logger("local_planner"); }
}  // namespace

TebLocalPlanner::TebLocalPlanner() {}

TebLocalPlanner::~TebLocalPlanner() {}

roborts_common::ErrorInfo TebLocalPlanner::ComputeVelocityCommands(roborts_msgs::msg::TwistAccel &cmd_vel) {
  if (!is_initialized_) {
    RCLCPP_ERROR(kLocalPlannerLogger(), "timed_elastic_band doesn't be initialized");
    roborts_common::ErrorInfo algorithm_init_error(roborts_common::LP_ALGORITHM_INITILIZATION_ERROR,
                                                   "teb initialize failed");
    RCLCPP_ERROR(kLocalPlannerLogger(), "%s", algorithm_init_error.error_msg().c_str());
    return algorithm_init_error;
  }

  GetPlan(temp_plan_);

  cmd_vel.twist.linear.x = 0;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = 0;

  cmd_vel.accel.linear.x = 0;
  cmd_vel.accel.linear.y = 0;
  cmd_vel.accel.angular.z = 0;

  UpdateRobotPose();
  UpdateRobotVel();
  UpdateGlobalToPlanTranform();

  auto time_now = std::chrono::system_clock::now();
  oscillation_time_ =
      std::chrono::duration_cast<std::chrono::milliseconds>(time_now - oscillation_).count() / 1000.0f;

  if (oscillation_time_ > 1.0) {
    if ((robot_pose_.GetPosition() - last_robot_pose_.GetPosition()).norm() < 0.1) {
      local_cost_.lock()->ClearCostMap();
    } else {
      oscillation_time_ = 0;
      oscillation_ = std::chrono::system_clock::now();
      last_robot_pose_ = robot_pose_;
    }
  }

  if (IsGoalReached()) {
    roborts_common::ErrorInfo algorithm_ok(roborts_common::OK, "reached the goal");
    RCLCPP_INFO(kLocalPlannerLogger(), "reached the goal");
    return algorithm_ok;
  }

  PruneGlobalPlan();

  int goal_idx = 0;

  if (!TransformGlobalPlan(&goal_idx)) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "plan transform error");
    RCLCPP_ERROR(kLocalPlannerLogger(), "%s", PlanTransformError.error_msg().c_str());
    return PlanTransformError;
  }

  if (transformed_plan_.empty()) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "transformed plan is empty");
    RCLCPP_ERROR(kLocalPlannerLogger(), "transformed plan is empty");
    return PlanTransformError;
  }

  if (global_plan_overwrite_orientation_) {
    transformed_plan_.back().SetTheta(EstimateLocalGoalOrientation(transformed_plan_.back(), goal_idx));
  }

  if (transformed_plan_.size() == 1) {
    transformed_plan_.insert(transformed_plan_.begin(), robot_pose_);
  } else {
    transformed_plan_.front() = robot_pose_;
  }

  obst_vector_.clear();

  robot_goal_ = transformed_plan_.back();
  UpdateObstacleWithCostmap(robot_goal_.GetPosition());

  UpdateViaPointsContainer();

  bool micro_control = false;
  if (global_plan_.poses.back().pose.position.z == 1) {
    micro_control = true;
  }

  bool success = optimal_->Optimal(transformed_plan_, &robot_current_vel_, free_goal_vel_, micro_control);

  if (!success) {
    optimal_->ClearPlanner();
    roborts_common::ErrorInfo OptimalError(roborts_common::LP_OPTIMAL_ERROR, "optimal error");
    RCLCPP_ERROR(kLocalPlannerLogger(), "optimal error");
    last_cmd_ = cmd_vel;
    return OptimalError;
  }

  bool feasible =
      optimal_->IsTrajectoryFeasible(teb_error_info_, robot_cost_.get(), robot_footprint_, robot_inscribed_radius_,
                                     robot_circumscribed_radius, fesiable_step_look_ahead_);
  if (!feasible) {
    optimal_->ClearPlanner();
    last_cmd_ = cmd_vel;
    RCLCPP_ERROR(kLocalPlannerLogger(), "trajectory is not feasible");
    roborts_common::ErrorInfo trajectory_error(roborts_common::LP_ALGORITHM_TRAJECTORY_ERROR,
                                              "trajectory is not feasible");
    return trajectory_error;
  }

  if (!optimal_->GetVelocity(teb_error_info_, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                             cmd_vel.accel.linear.x, cmd_vel.accel.linear.y, cmd_vel.accel.angular.z)) {
    optimal_->ClearPlanner();
    RCLCPP_ERROR(kLocalPlannerLogger(), "can not get the velocity");
    roborts_common::ErrorInfo velocity_error(roborts_common::LP_VELOCITY_ERROR, "velocity is not feasible");
    last_cmd_ = cmd_vel;
    return velocity_error;
  }

  SaturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, max_vel_x_, max_vel_y_,
                   max_vel_theta_, max_vel_x_backwards);

  last_cmd_ = cmd_vel;

  optimal_->Visualize();

  RCLCPP_INFO(kLocalPlannerLogger(), "compute velocity succeed");
  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

bool TebLocalPlanner::IsGoalReached() {
  geometry_msgs::msg::PoseStamped goal_plan = global_plan_.poses.back();
  geometry_msgs::msg::PoseStamped goal_global;
  tf2::doTransform(goal_plan, goal_global, plan_to_global_transform_);
  auto goal = DataConverter::LocalConvertGData(goal_global.pose);

  auto distance = (goal.first - robot_pose_.GetPosition()).norm();
  double delta_orient = g2o::normalize_theta(goal.second - robot_pose_.GetTheta());

  if (distance < xy_goal_tolerance_ && fabs(delta_orient) < yaw_goal_tolerance_) {
    RCLCPP_INFO(kLocalPlannerLogger(), "goal reached");
    return true;
  }
  return false;
}

bool TebLocalPlanner::SetPlan(const nav_msgs::msg::Path &plan, const geometry_msgs::msg::PoseStamped &goal) {
  if (plan_mutex_.try_lock()) {
    RCLCPP_INFO(kLocalPlannerLogger(), "set plan");
    if (plan.poses.empty()) {
      temp_plan_.poses.push_back(goal);
    } else {
      temp_plan_ = plan;
    }
    plan_mutex_.unlock();
  }
  return true;
}

bool TebLocalPlanner::GetPlan(const nav_msgs::msg::Path &plan) {
  (void)plan;
  if (plan_mutex_.try_lock()) {
    global_plan_ = temp_plan_;
    plan_mutex_.unlock();
  }
  return true;
}

bool TebLocalPlanner::PruneGlobalPlan() {
  if (global_plan_.poses.empty()) {
    return true;
  }
  auto tf_buf = tf_.lock();
  if (!tf_buf) {
    return false;
  }
  try {
    geometry_msgs::msg::PoseStamped robot_plan;
    tf_buf->transform(robot_tf_pose_, robot_plan, global_plan_.poses.front().header.frame_id,
                      tf2::durationFromSec(0.5));
    double ox = robot_plan.pose.position.x;
    double oy = robot_plan.pose.position.y;

    for (auto iterator = global_plan_.poses.begin(); iterator != global_plan_.poses.end(); ++iterator) {
      Eigen::Vector2d temp_vector(ox - iterator->pose.position.x, oy - iterator->pose.position.y);
      if (temp_vector.norm() < 0.8) {
        if (iterator == global_plan_.poses.begin()) {
          break;
        }
        global_plan_.poses.erase(global_plan_.poses.begin(), iterator);
        break;
      }
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(kLocalPlannerLogger(), "prune global plan false, %s", ex.what());
    return false;
  }
  return true;
}

bool TebLocalPlanner::TransformGlobalPlan(int *current_goal_idx) {
  transformed_plan_.clear();

  auto tf_buf = tf_.lock();
  if (!tf_buf) {
    return false;
  }

  try {
    if (global_plan_.poses.empty()) {
      RCLCPP_ERROR(kLocalPlannerLogger(), "Received plan with zero length");
      if (current_goal_idx) {
        *current_goal_idx = 0;
      }
      return false;
    }
    UpdateGlobalToPlanTranform();

    geometry_msgs::msg::PoseStamped robot_pose;
    tf_buf->transform(robot_tf_pose_, robot_pose, global_plan_.poses.front().header.frame_id,
                      tf2::durationFromSec(0.5));
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;

    double dist_threshold = std::max(costmap_->GetSizeXCell() * costmap_->GetResolution() / 2.0,
                                     costmap_->GetSizeYCell() * costmap_->GetResolution() / 2.0);
    dist_threshold *= 0.85;

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    double new_sq_dist = 0;
    while (i < (int)global_plan_.poses.size()) {
      double x_diff = robot_x - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_y - global_plan_.poses[i].pose.position.y;
      new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {
        sq_dist = new_sq_dist;
        break;
      }
      sq_dist = new_sq_dist;
      ++i;
    }

    geometry_msgs::msg::PoseStamped newer_pose;

    double plan_length = 0;

    while (i < (int)global_plan_.poses.size() && sq_dist <= sq_dist_threshold &&
           (cut_lookahead_dist_ <= 0 || plan_length <= cut_lookahead_dist_)) {
      const geometry_msgs::msg::PoseStamped pose = global_plan_.poses[i];

      geometry_msgs::msg::PoseStamped pose_global;
      tf2::doTransform(pose, pose_global, plan_to_global_transform_);

      auto temp = DataConverter::LocalConvertGData(pose_global.pose);
      DataBase data_pose(temp.first, temp.second);

      transformed_plan_.push_back(data_pose);

      double x_diff = robot_x - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_y - global_plan_.poses[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (i > 0 && cut_lookahead_dist_ > 0) {
        plan_length +=
            Distance(global_plan_.poses[i - 1].pose.position.x, global_plan_.poses[i - 1].pose.position.y,
                     global_plan_.poses[i].pose.position.x, global_plan_.poses[i].pose.position.y);
      }

      ++i;
    }

    if (transformed_plan_.empty()) {
      geometry_msgs::msg::PoseStamped pose_back = global_plan_.poses.back();
      geometry_msgs::msg::PoseStamped pose_global;
      tf2::doTransform(pose_back, pose_global, plan_to_global_transform_);

      auto temp = DataConverter::LocalConvertGData(pose_global.pose);
      transformed_plan_.push_back(DataBase(temp.first, temp.second));

      if (current_goal_idx) {
        *current_goal_idx = int(global_plan_.poses.size()) - 1;
      }
    } else {
      if (current_goal_idx) {
        *current_goal_idx = i - 1;
      }
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(kLocalPlannerLogger(), "Transform error: %s", ex.what());
    return false;
  }

  return true;
}

double TebLocalPlanner::EstimateLocalGoalOrientation(const DataBase &local_goal, int current_goal_idx,
                                                    int moving_average_length) const {
  int n = (int)global_plan_.poses.size();

  if (current_goal_idx > n - moving_average_length - 2) {
    if (current_goal_idx >= n - 1) {
      return local_goal.GetTheta();
    }
    tf2::Quaternion q_plan;
    tf2::fromMsg(plan_to_global_transform_.transform.rotation, q_plan);
    tf2::Quaternion q_back;
    tf2::fromMsg(global_plan_.poses.back().pose.orientation, q_back);
    return tf2::getYaw(q_plan * q_back);
  }

  moving_average_length = std::min(moving_average_length, n - current_goal_idx - 1);

  std::vector<double> candidates;

  geometry_msgs::msg::PoseStamped pose_k_plan = global_plan_.poses.at(static_cast<size_t>(current_goal_idx));
  geometry_msgs::msg::PoseStamped g_curr;
  tf2::doTransform(pose_k_plan, g_curr, plan_to_global_transform_);

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i) {
    const geometry_msgs::msg::PoseStamped pose_kp1_plan = global_plan_.poses.at(static_cast<size_t>(i + 1));
    geometry_msgs::msg::PoseStamped g_next;
    tf2::doTransform(pose_kp1_plan, g_next, plan_to_global_transform_);

    candidates.push_back(std::atan2(g_next.pose.position.y - g_curr.pose.position.y,
                                     g_next.pose.position.x - g_curr.pose.position.x));

    if (i < range_end - 1) {
      g_curr = g_next;
    }
  }
  return AverageAngles(candidates);
}

void TebLocalPlanner::UpdateObstacleWithCostmap(Eigen::Vector2d local_goal) {
  Eigen::Vector2d goal_orient = local_goal - robot_pose_.GetPosition();

  for (unsigned int i = 0; i < costmap_->GetSizeXCell() - 1; ++i) {
    for (unsigned int j = 0; j < costmap_->GetSizeYCell() - 1; ++j) {
      if (costmap_->GetCost(i, j) == roborts_local_planner::LETHAL_OBSTACLE) {
        Eigen::Vector2d obs;
        costmap_->Map2World(i, j, obs.coeffRef(0), obs.coeffRef(1));

        Eigen::Vector2d obs_dir = obs - robot_pose_.GetPosition();
        if (obs_dir.dot(goal_orient) < 0 && obs_dir.norm() > osbtacle_behind_robot_dist_) {
          continue;
        }

        obst_vector_.push_back(ObstaclePtr(new PointObstacle(obs)));
      }
    }
  }
}

void TebLocalPlanner::UpdateViaPointsContainer() {
  via_points_.clear();

  double min_separation = param_config_.trajectory_opt().global_plan_viapoint_sep();
  if (min_separation < 0) {
    return;
  }

  std::size_t prev_idx = 0;
  for (std::size_t i = 1; i < transformed_plan_.size(); ++i) {
    if (Distance(transformed_plan_[prev_idx].GetPosition().coeff(0), transformed_plan_[prev_idx].GetPosition().coeff(1),
                 transformed_plan_[i].GetPosition().coeff(0), transformed_plan_[i].GetPosition().coeff(1)) <
        min_separation) {
      continue;
    }

    via_points_.push_back(transformed_plan_[i].GetPosition());
    prev_idx = i;
  }
}

void TebLocalPlanner::UpdateRobotPose() {
  local_cost_.lock()->GetRobotPose(robot_tf_pose_);
  Eigen::Vector2d position;
  position.coeffRef(0) = robot_tf_pose_.pose.position.x;
  position.coeffRef(1) = robot_tf_pose_.pose.position.y;
  robot_pose_ = DataBase(position, tf2::getYaw(robot_tf_pose_.pose.orientation));
}

void TebLocalPlanner::UpdateRobotVel() {
  odom_info_.GetVel(robot_current_vel_);
}

void TebLocalPlanner::UpdateGlobalToPlanTranform() {
  auto tf_buf = tf_.lock();
  if (!tf_buf || global_plan_.poses.empty()) {
    return;
  }
  try {
    plan_to_global_transform_ =
        tf_buf->lookupTransform(global_frame_, global_plan_.poses.front().header.frame_id, tf2::TimePointZero,
                                tf2::durationFromSec(0.5));
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(kLocalPlannerLogger(), "%s", ex.what());
  }
}

void TebLocalPlanner::SaturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y,
                                         double max_vel_theta, double max_vel_x_backwards) const {
  if (vx > max_vel_x) {
    vx = max_vel_x;
  }

  if (max_vel_x_backwards <= 0) {
    RCLCPP_INFO(kLocalPlannerLogger(),
                "Do not choose max_vel_x_backwards to be <=0. "
                "Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  } else if (vx < -max_vel_x_backwards) {
    vx = -max_vel_x_backwards;
  }

  if (vy > max_vel_y) {
    vy = max_vel_y;
  } else if (vy < -max_vel_y) {
    vy = -max_vel_y;
  }

  if (omega > max_vel_theta) {
    omega = max_vel_theta;
  } else if (omega < -max_vel_theta) {
    omega = -max_vel_theta;
  }
}

double TebLocalPlanner::ConvertTransRotVelToSteeringAngle(double v, double omega, double wheelbase,
                                                           double min_turning_radius) const {
  if (omega == 0 || v == 0) {
    return 0;
  }

  double radius = v / omega;

  if (fabs(radius) < min_turning_radius) {
    radius = double(g2o::sign(radius)) * min_turning_radius;
  }

  return std::atan(wheelbase / radius);
}

roborts_common::ErrorInfo TebLocalPlanner::Initialize(std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
                                                        std::shared_ptr<tf2_ros::Buffer> tf,
                                                        LocalVisualizationPtr visual) {
  if (!is_initialized_) {
    oscillation_ = std::chrono::system_clock::now();
    tf_ = tf;
    local_cost_ = local_cost;

    std::string full_path = ament_index_cpp::get_package_share_directory("roborts_planning") +
                           "/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt";
    if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_)) {
      RCLCPP_ERROR(kLocalPlannerLogger(), "error occur when loading config file");
      roborts_common::ErrorInfo read_file_error(roborts_common::ErrorCode::LP_ALGORITHM_INITILIZATION_ERROR,
                                                "load algorithm param file failed");
      is_initialized_ = false;
      RCLCPP_ERROR(kLocalPlannerLogger(), "%s", read_file_error.error_msg().c_str());
      return read_file_error;
    }

    max_vel_x_ = param_config_.kinematics_opt().max_vel_x();
    max_vel_y_ = param_config_.kinematics_opt().max_vel_y();
    max_vel_theta_ = param_config_.kinematics_opt().max_vel_theta();
    max_vel_x_backwards = param_config_.kinematics_opt().max_vel_x_backwards();
    free_goal_vel_ = param_config_.tolerance_opt().free_goal_vel();

    xy_goal_tolerance_ = param_config_.tolerance_opt().xy_goal_tolerance();
    yaw_goal_tolerance_ = param_config_.tolerance_opt().yaw_goal_tolerance();

    cut_lookahead_dist_ = param_config_.trajectory_opt().max_global_plan_lookahead_dist();
    fesiable_step_look_ahead_ = param_config_.trajectory_opt().feasibility_check_no_poses();

    osbtacle_behind_robot_dist_ = param_config_.obstacles_opt().costmap_obstacles_behind_robot_dist();

    global_plan_overwrite_orientation_ = param_config_.trajectory_opt().global_plan_overwrite_orientation();

    global_frame_ = local_cost_.lock()->GetGlobalFrameID();

    visual_ = visual;

    costmap_ = local_cost_.lock()->GetLayeredCostmap()->GetCostMap();

    robot_cost_ = std::make_shared<roborts_local_planner::RobotPositionCost>(*costmap_);

    obst_vector_.reserve(200);
    RobotFootprintModelPtr robot_model = GetRobotFootprintModel(param_config_);
    optimal_ = OptimalBasePtr(new TebOptimal(param_config_, &obst_vector_, robot_model, visual_, &via_points_));

    roborts_costmap::RobotPose robot_pose;
    local_cost_.lock()->GetRobotPose(robot_pose);
    auto temp_pose = DataConverter::LocalConvertRMData(robot_pose);
    robot_pose_ = DataBase(temp_pose.first, temp_pose.second);
    robot_footprint_.push_back(robot_pose_.GetPosition());
    last_robot_pose_ = robot_pose_;

    RobotPositionCost::CalculateMinAndMaxDistances(robot_footprint_, robot_inscribed_radius_,
                                                    robot_circumscribed_radius);

    auto node_sh = local_cost_.lock()->GetNode();
    odom_info_.SetNode(node_sh);
    odom_info_.SetTopic(param_config_.opt_frame().odom_frame());

    is_initialized_ = true;
    RCLCPP_INFO(kLocalPlannerLogger(), "local algorithm initialize ok");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
  }

  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

void TebLocalPlanner::RegisterErrorCallBack(ErrorInfoCallback error_callback) { error_callback_ = error_callback; }

bool TebLocalPlanner::CutAndTransformGlobalPlan(int *current_goal_idx) {
  (void)current_goal_idx;
  if (!transformed_plan_.empty()) {
    transformed_plan_.clear();
  }
  return false;
}

bool TebLocalPlanner::SetPlanOrientation() {
  if (global_plan_.poses.size() < 2) {
    RCLCPP_WARN(kLocalPlannerLogger(),
                "can not compute the orientation because the global plan size is: %ld",
                (long)global_plan_.poses.size());
    return false;
  }
  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i) {
    double x = global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x;
    double y = global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y;
    double angle = atan2(y, x);
    auto quaternion = EulerToQuaternion(0, 0, angle);
    global_plan_.poses[i].pose.orientation.w = quaternion[0];
    global_plan_.poses[i].pose.orientation.x = quaternion[1];
    global_plan_.poses[i].pose.orientation.y = quaternion[2];
    global_plan_.poses[i].pose.orientation.z = quaternion[3];
  }
  return true;
}

}  // namespace roborts_local_planner







