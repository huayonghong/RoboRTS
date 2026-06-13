#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer_interface.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "costmap_parameter_setting.pb.h"
#include "costmap_interface.h"

namespace roborts_costmap {


CostmapInterface::CostmapInterface(std::string map_name,
                                   rclcpp::Node::SharedPtr node,
                                   tf2_ros::Buffer &tf_buffer,
                                   std::string config_file)
    : layered_costmap_(nullptr),
      name_(std::move(map_name)),
      config_file_(std::move(config_file)),
      node_(std::move(node)),
      tf_(tf_buffer),
      stop_updates_(false),
      initialized_(true),
      stopped_(false),
      robot_stopped_(false),
      map_update_thread_(nullptr),
      last_publish_(node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME)),
      dist_behind_robot_threshold_to_care_obstacles_(0.05),
      is_debug_(false),
      map_update_thread_shutdown_(false),
      footprint_padding_(0),
      unpadded_footprint_(),
      padded_footprint_(),
      timer_(nullptr) {
  LoadParameter();
  layered_costmap_ = new CostmapLayers(global_frame_, is_rolling_window_, is_track_unknown_);
  layered_costmap_->SetFilePath(config_file_inflation_);

  std::string tf_err;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() &&
         std::chrono::steady_clock::now() < deadline) {
    if (tf_.canTransform(global_frame_, robot_base_frame_, tf2::TimePointZero,
                         tf2::durationFromSec(0.05), &tf_err)) {
      break;
    }
    tf_err.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (has_static_layer_) {
    auto *plugin_static_layer = new StaticLayer;
    layered_costmap_->AddPlugin(plugin_static_layer);
    plugin_static_layer->Initialize(layered_costmap_, name_ + "/static_layer",
                                    &tf_, node_);
  }
  if (has_obstacle_layer_) {
    auto *plugin_obstacle_layer = new ObstacleLayer;
    layered_costmap_->AddPlugin(plugin_obstacle_layer);
    plugin_obstacle_layer->Initialize(layered_costmap_,
                                      name_ + "/obstacle_layer", &tf_, node_);
  }
  Layer *plugin_inflation_layer = new InflationLayer;
  layered_costmap_->AddPlugin(plugin_inflation_layer);
  plugin_inflation_layer->Initialize(layered_costmap_, name_ + "/inflation_layer",
                                     &tf_, node_);

  SetUnpaddedRobotFootprint(footprint_points_);

  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;
  robot_stopped_ = false;
  map_update_thread_shutdown_ = false;

  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&CostmapInterface::DetectMovement, this));

  if (is_debug_) {
    cost_translation_table_[0] = 0;
    cost_translation_table_[253] = 99;
    cost_translation_table_[254] = 100;
    cost_translation_table_[255] = -1;
    for (int i = 1; i < 253; i++) {
      cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
    }
  }

  costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      name_ + "/costmap", rclcpp::QoS(10));
  map_update_thread_ =
      new std::thread(std::bind(&CostmapInterface::MapUpdateLoop, this, map_update_frequency_));
  if (is_rolling_window_) {
    layered_costmap_->ResizeMap((unsigned int)(map_width_ / map_resolution_),
                                (unsigned int)(map_height_ / map_resolution_),
                                map_resolution_,
                                map_origin_x_,
                                map_origin_y_);
  }
}

void CostmapInterface::LoadParameter() {
  ParaCollection ParaCollectionConfig;
  roborts_common::ReadProtoFromTextFile(config_file_.c_str(), &ParaCollectionConfig);

  config_file_ = ament_index_cpp::get_package_share_directory("roborts_costmap") +
      ParaCollectionConfig.para_costmap_interface().inflation_file_path();

  map_update_frequency_ =
      ParaCollectionConfig.para_costmap_interface().map_update_frequency();
  global_frame_ = ParaCollectionConfig.para_costmap_interface().global_frame();
  robot_base_frame_ = ParaCollectionConfig.para_costmap_interface().robot_base_frame();
  footprint_padding_ = ParaCollectionConfig.para_costmap_interface().footprint_padding();
  transform_tolerance_ =
      ParaCollectionConfig.para_costmap_interface().transform_tolerance();
  is_rolling_window_ = ParaCollectionConfig.para_costmap_interface().is_rolling_window();
  is_debug_ = ParaCollectionConfig.para_basic().is_debug();
  is_track_unknown_ = ParaCollectionConfig.para_costmap_interface().is_tracking_unknown();
  has_obstacle_layer_ = ParaCollectionConfig.para_costmap_interface().has_obstacle_layer();
  has_static_layer_ = ParaCollectionConfig.para_costmap_interface().has_static_layer();
  map_width_ = ParaCollectionConfig.para_costmap_interface().map_width();
  map_height_ = ParaCollectionConfig.para_costmap_interface().map_height();
  map_origin_x_ = ParaCollectionConfig.para_costmap_interface().map_origin_x();
  map_origin_y_ = ParaCollectionConfig.para_costmap_interface().map_origin_y();
  map_resolution_ = ParaCollectionConfig.para_costmap_interface().map_resolution();

  config_file_inflation_ = ament_index_cpp::get_package_share_directory("roborts_costmap") +
      ParaCollectionConfig.para_costmap_interface().inflation_file_path();

  geometry_msgs::msg::Point point;
  for (int i = 0; i < ParaCollectionConfig.footprint().point().size(); ++i) {
    point.x = ParaCollectionConfig.footprint().point(i).x();
    point.y = ParaCollectionConfig.footprint().point(i).y();
    point.z = 0.0;
    footprint_points_.push_back(point);
  }
}

void CostmapInterface::SetUnpaddedRobotFootprintPolygon(
    const geometry_msgs::msg::Polygon &footprint) {
  SetUnpaddedRobotFootprint(ToPointVector(footprint));
}

CostmapInterface::~CostmapInterface() {
  if (timer_) {
    timer_->cancel();
  }
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != nullptr) {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  delete layered_costmap_;
}

void CostmapInterface::SetUnpaddedRobotFootprint(
    const std::vector<geometry_msgs::msg::Point> &points) {
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  PadFootprint(padded_footprint_, footprint_padding_);
  layered_costmap_->SetFootprint(padded_footprint_);
}

void CostmapInterface::DetectMovement() {
  geometry_msgs::msg::PoseStamped new_pose;
  if (!GetRobotPose(new_pose)) {
    robot_stopped_ = false;
    return;
  }
  if (old_pose_.header.frame_id.empty()) {
    old_pose_ = new_pose;
    robot_stopped_ = false;
    return;
  }
  auto dist = std::hypot(new_pose.pose.position.x - old_pose_.pose.position.x,
                         new_pose.pose.position.y - old_pose_.pose.position.y);
  double yaw_new = tf2::getYaw(new_pose.pose.orientation);
  double yaw_old = tf2::getYaw(old_pose_.pose.orientation);
  double yaw_diff = yaw_new - yaw_old;
  while (yaw_diff > M_PI)
    yaw_diff -= 2 * M_PI;
  while (yaw_diff < -M_PI)
    yaw_diff += 2 * M_PI;
  if ((std::fabs(dist) + std::fabs(yaw_diff)) < 1e-3) {
    old_pose_ = new_pose;
    robot_stopped_ = true;
  } else {
    old_pose_ = new_pose;
    robot_stopped_ = false;
  }
}

void CostmapInterface::MapUpdateLoop(double frequency) {
  if (frequency <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Frequency must be positive in MapUpdateLoop.");
    return;
  }
  rclcpp::WallRate loop_rate(frequency);

  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    UpdateMap();
    loop_rate.sleep();

    double x{};
    double y{};
    Costmap2D *temp_costmap = layered_costmap_->GetCostMap();
    unsigned char *data = temp_costmap->GetCharMap();
    temp_costmap->Map2World(0, 0, x, y);
    grid_.header.frame_id = global_frame_;
    grid_.header.stamp = node_->now();
    if (is_rolling_window_) {
      grid_.info.resolution = map_resolution_;
      grid_.info.width = map_width_ / map_resolution_;
      grid_.info.height = map_height_ / map_resolution_;
      grid_.info.origin.position.x = x - map_resolution_ * 0.5;
      grid_.info.origin.position.y = y - map_resolution_ * 0.5;
      grid_.info.origin.position.z = 0;
      grid_.info.origin.orientation.w = 1.0;
      grid_.data.resize(grid_.info.width * grid_.info.height);
    } else {
      auto resolution = temp_costmap->GetResolution();
      auto map_width_cell = temp_costmap->GetSizeXCell();
      auto map_height_cell = temp_costmap->GetSizeYCell();
      grid_.info.resolution = resolution;
      grid_.info.width = map_width_cell;
      grid_.info.height = map_height_cell;
      grid_.info.origin.position.x = temp_costmap->GetOriginX();
      grid_.info.origin.position.y = temp_costmap->GetOriginY();
      grid_.info.origin.position.z = 0;
      grid_.info.origin.orientation.w = 1.0;
      grid_.data.resize(map_width_cell * map_height_cell);
    }
    for (size_t i = 0; i < grid_.data.size(); ++i) {
      grid_.data[i] = cost_translation_table_[data[i]];
    }
    costmap_pub_->publish(grid_);
  }
}

void CostmapInterface::UpdateMap() {
  if (!stop_updates_) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    if (GetRobotPose(pose_stamped)) {
      double x = pose_stamped.pose.position.x;
      double y = pose_stamped.pose.position.y;
      double yaw = tf2::getYaw(pose_stamped.pose.orientation);
      layered_costmap_->UpdateMap(x, y, yaw);
      initialized_ = true;
    }
  }
}

void CostmapInterface::Start() {
  auto *plugins = layered_costmap_->GetPlugins();
  if (stopped_) {
    for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
      (*plugin)->Activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;
  rclcpp::WallRate r(100.0);
  while (rclcpp::ok() && !initialized_) {
    r.sleep();
  }
}

void CostmapInterface::Stop() {
  stop_updates_ = true;
  auto *plugins = layered_costmap_->GetPlugins();
  for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
    (*plugin)->Deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void CostmapInterface::Pause() {
  stop_updates_ = true;
  initialized_ = false;
}

void CostmapInterface::Resume() {
  stop_updates_ = false;
  rclcpp::WallRate r(100.0);
  while (!initialized_) {
    r.sleep();
    if (!rclcpp::ok()) {
      break;
    }
  }
}

void CostmapInterface::ResetLayers() {
  Costmap2D *master = layered_costmap_->GetCostMap();
  master->ResetPartMap(0, 0, master->GetSizeXCell(), master->GetSizeYCell());
  auto plugins = layered_costmap_->GetPlugins();
  for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
    (*plugin)->Reset();
  }
}

bool CostmapInterface::GetRobotPose(
    geometry_msgs::msg::PoseStamped &global_pose) const {
  geometry_msgs::msg::PoseStamped robot_base_pose;
  robot_base_pose.header.frame_id = robot_base_frame_;
  robot_base_pose.header.stamp.sec = 0;
  robot_base_pose.header.stamp.nanosec = 0;
  robot_base_pose.pose.orientation.w = 1.0;

  try {
    tf_.transform(robot_base_pose, global_pose, global_frame_,
                  tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_DEBUG(node_->get_logger(), "%s", ex.what());
    return false;
  }
}

void CostmapInterface::GetOrientedFootprint(
    std::vector<geometry_msgs::msg::Point> &oriented_footprint) const {
  geometry_msgs::msg::PoseStamped global_pose;
  if (!GetRobotPose(global_pose)) {
    return;
  }
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  TransformFootprint(global_pose.pose.position.x, global_pose.pose.position.y,
                       yaw, padded_footprint_, oriented_footprint);
}

void CostmapInterface::GetFootprint(std::vector<Eigen::Vector3f> &footprint) {
  std::vector<geometry_msgs::msg::Point> ros_footprint = GetRobotFootprint();
  Eigen::Vector3f pt;
  for (const auto &it : ros_footprint) {
    pt << it.x, it.y, it.z;
    footprint.push_back(pt);
  }
}

void CostmapInterface::GetOrientedFootprint(std::vector<Eigen::Vector3f> &footprint) {
  std::vector<geometry_msgs::msg::Point> oriented_fp;
  GetOrientedFootprint(oriented_fp);
  Eigen::Vector3f position;
  for (const auto &it : oriented_fp) {
    position << it.x, it.y, it.z;
    footprint.push_back(position);
  }
}

bool CostmapInterface::GetRobotPose(RobotPose &pose) {
  geometry_msgs::msg::PoseStamped ps;
  if (!GetRobotPose(ps)) {
    return false;
  }
  pose.time = rclcpp::Time(ps.header.stamp);
  pose.frame_id = ps.header.frame_id;
  pose.position << ps.pose.position.x, ps.pose.position.y,
      ps.pose.position.z;

  Eigen::Quaternionf ei(ps.pose.orientation.w, ps.pose.orientation.x,
                         ps.pose.orientation.y, ps.pose.orientation.z);
  pose.rotation = ei.normalized().toRotationMatrix();
  return true;
}

unsigned char *CostmapInterface::GetCharMap() const {
  return layered_costmap_->GetCostMap()->GetCharMap();
}

geometry_msgs::msg::PoseStamped CostmapInterface::Pose2GlobalFrame(
    const geometry_msgs::msg::PoseStamped &pose_msg) {
  geometry_msgs::msg::PoseStamped in = pose_msg;
  in.header.stamp.sec = 0;
  in.header.stamp.nanosec = 0;
  geometry_msgs::msg::PoseStamped out;
  try {
    tf_.transform(in, out, global_frame_,
                  tf2::durationFromSec(transform_tolerance_));
    return out;
  } catch (const tf2::TransformException &) {
    return pose_msg;
  }
}

void CostmapInterface::ClearCostMap() {
  std::vector<Layer *> *plugins = layered_costmap_->GetPlugins();
  geometry_msgs::msg::PoseStamped pose;
  if (!GetRobotPose(pose)) {
    return;
  }
  double pose_x = pose.pose.position.x;
  double pose_y = pose.pose.position.y;

  for (auto plugin_iter = plugins->begin(); plugin_iter != plugins->end(); ++plugin_iter) {
    roborts_costmap::Layer *plugin = *plugin_iter;
    if (plugin->GetName().find("obstacle") != std::string::npos) {
      ClearLayer(reinterpret_cast<CostmapLayer *>(plugin), pose_x, pose_y);
    }
  }
}

void CostmapInterface::ClearLayer(CostmapLayer *costmap_layer_ptr, double pose_x,
                                   double pose_y) {
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_layer_ptr->GetMutex()));
  double reset_distance = 0.1;
  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  int start_x{};
  int start_y{};
  int end_x{};
  int end_y{};
  costmap_layer_ptr->World2MapNoBoundary(start_point_x, start_point_y,
                                         start_x, start_y);
  costmap_layer_ptr->World2MapNoBoundary(end_point_x, end_point_y,
                                         end_x, end_y);

  unsigned char *grid = costmap_layer_ptr->GetCharMap();
  for (unsigned int x = 0; x < costmap_layer_ptr->GetSizeXCell(); x++) {
    bool xrange =
        static_cast<int>(x) > start_x && static_cast<int>(x) < end_x;
    for (unsigned int y = 0; y < costmap_layer_ptr->GetSizeYCell(); y++) {
      if (xrange && static_cast<int>(y) > start_y &&
          static_cast<int>(y) < end_y) {
        continue;
      }
      int index =
          costmap_layer_ptr->GetIndex(static_cast<int>(x), static_cast<int>(y));
      if (grid[index] != NO_INFORMATION) {
        grid[index] = NO_INFORMATION;
      }
    }
  }

  double ox = costmap_layer_ptr->GetOriginX(),
         oy = costmap_layer_ptr->GetOriginY();
  double width = costmap_layer_ptr->GetSizeXWorld(),
         height = costmap_layer_ptr->GetSizeYWorld();
  costmap_layer_ptr->AddExtraBounds(ox, oy, ox + width, oy + height);
}

}  // namespace roborts_costmap
