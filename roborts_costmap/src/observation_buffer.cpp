/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 ***************************************************************************/

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logging.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/common/transforms.h>

#include <string>
#include <vector>

#include "observation_buffer.h"

namespace roborts_costmap {

namespace {

inline builtin_interfaces::msg::Time pclStampToBuiltin(uint64_t stamp_usec) {
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(stamp_usec / 1000000ULL);
  msg.nanosec = static_cast<uint32_t>((stamp_usec % 1000000ULL) * 1000ULL);
  return msg;
}

}  // namespace

ObservationBuffer::ObservationBuffer(std::string topic_name,
                                     double observation_keep_time,
                                     double expected_update_rate,
                                     double min_obstacle_height,
                                     double max_obstacle_height,
                                     double obstacle_range,
                                     double raytrace_range,
                                     tf2_ros::Buffer &tf,
                                     std::string global_frame,
                                     std::string sensor_frame,
                                     double tf_tolerance,
                                     const rclcpp::Clock::SharedPtr &clock)
    : tf_(tf),
      observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),
      expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),
      last_updated_(clock->now()),
      global_frame_(std::move(global_frame)),
      sensor_frame_(std::move(sensor_frame)),
      topic_name_(std::move(topic_name)),
      min_obstacle_height_(min_obstacle_height),
      max_obstacle_height_(max_obstacle_height),
      obstacle_range_(obstacle_range),
      raytrace_range_(raytrace_range),
      tf_tolerance_(tf_tolerance),
      clock_(clock) {}

ObservationBuffer::~ObservationBuffer() = default;

bool ObservationBuffer::SetGlobalFrame(const std::string new_global_frame)
{
  rclcpp::Time transform_time = clock_->now();

  geometry_msgs::msg::TransformStamped tf_global_old_to_new;
  try {
    tf_global_old_to_new =
        tf_.lookupTransform(new_global_frame, global_frame_,
                            tf2_ros::fromRcl(transform_time), tf2::durationFromSec(tf_tolerance_));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("costmap"),
                 "Transform between %s and %s failed: %s",
                 new_global_frame.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }

  for (Observation &obs : observation_list_) {
    try {
      geometry_msgs::msg::PointStamped origin_in;
      origin_in.header.frame_id = global_frame_;
      origin_in.header.stamp = transform_time;
      origin_in.point = obs.origin_;
      geometry_msgs::msg::PointStamped origin_out;
      tf2::doTransform(origin_in, origin_out, tf_global_old_to_new);
      obs.origin_ = origin_out.point;

      Eigen::Isometry3d iso = tf2::transformToEigen(tf_global_old_to_new.transform);
      pcl::PointCloud<pcl::PointXYZ> transformed;
      pcl::transformPointCloud(*(obs.cloud_), transformed, Eigen::Affine3f(iso.matrix().cast<float>()));
      *(obs.cloud_) = transformed;
      obs.cloud_->header.frame_id = new_global_frame;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("costmap"),
                   "TF error transforming an observation from %s to %s: %s",
                   global_frame_.c_str(), new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::BufferCloud(const sensor_msgs::msg::PointCloud2 &cloud)
{
  try {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    BufferCloud(pcl_cloud);
  } catch (const pcl::PCLException &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("costmap"), "Failed to convert message to pcl: %s", ex.what());
    return;
  }
}

void ObservationBuffer::BufferCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in)
{
  observation_list_.push_front(Observation());

  std::string origin_frame =
      sensor_frame_.empty() ? std::string(cloud_in.header.frame_id) : sensor_frame_;

  rclcpp::Time cloud_time(pclStampToBuiltin(cloud_in.header.stamp), clock_->get_clock_type());

  Observation &front = observation_list_.front();

  try {
    geometry_msgs::msg::PoseStamped local_pose;
    local_pose.header.frame_id = origin_frame;
    local_pose.header.stamp = cloud_time;
    local_pose.pose.orientation.w = 1.;

    geometry_msgs::msg::PoseStamped pose_global =
        tf_.transform(local_pose, global_frame_, tf2::durationFromSec(0.5));

    front.origin_.x = pose_global.pose.position.x;
    front.origin_.y = pose_global.pose.position.y;
    front.origin_.z = pose_global.pose.position.z;

    front.raytrace_range_ = raytrace_range_;
    front.obstacle_range_ = obstacle_range_;

    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud_in, ros_cloud);
    ros_cloud.header.frame_id = origin_frame;
    ros_cloud.header.stamp = pclStampToBuiltin(cloud_in.header.stamp);

    geometry_msgs::msg::TransformStamped transform =
        tf_.lookupTransform(global_frame_, origin_frame, tf2_ros::fromRcl(cloud_time),
                            tf2::durationFromSec(0.5));

    sensor_msgs::msg::PointCloud2 ros_cloud_tf;
    tf2_sensor_msgs::doTransform(ros_cloud, ros_cloud_tf, transform);

    pcl::PointCloud<pcl::PointXYZ> global_frame_cloud;
    pcl::fromROSMsg(ros_cloud_tf, global_frame_cloud);
    *(front.cloud_) = global_frame_cloud;
    pcl_conversions::toPCL(ros_cloud_tf.header.stamp,
                           front.cloud_->header.stamp);
    front.cloud_->header.frame_id = ros_cloud_tf.header.frame_id.c_str();
  }
  catch (const tf2::TransformException &ex) {
    observation_list_.pop_front();
    RCLCPP_ERROR(rclcpp::get_logger("costmap"), "Observation buffer TF: %s", ex.what());
    return;
  }

  last_updated_ = clock_->now();
  PurgeStaleObservations();
}

void ObservationBuffer::GetObservations(std::vector<Observation> &observations)
{
  PurgeStaleObservations();
  for (const Observation &obs : observation_list_) {
    observations.push_back(obs);
  }
}

void ObservationBuffer::PurgeStaleObservations()
{
  if (!observation_list_.empty()) {
    if (observation_keep_time_.nanoseconds() == 0LL) {
      auto obs_it = observation_list_.begin();
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    for (auto obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
      Observation &obs = *obs_it;
      rclcpp::Time cloud_stamp(pclStampToBuiltin(obs.cloud_->header.stamp), clock_->get_clock_type());
      if ((last_updated_ - cloud_stamp) > observation_keep_time_) {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::IsCurrent() const
{
  if (expected_update_rate_.seconds() == 0.0) {
    return true;
  }
  double age = (clock_->now() - last_updated_).seconds();
  bool current = age <= expected_update_rate_.seconds();
  if (!current) {
    RCLCPP_WARN(
        rclcpp::get_logger("costmap"),
        "The %s observation buffer has not been updated for %.2f seconds, expected every %.2f seconds.",
        topic_name_.c_str(), age, expected_update_rate_.seconds());
  }
  return current;
}

void ObservationBuffer::ResetLastUpdated()
{
  last_updated_ = clock_->now();
}

} //namespace roborts_costmap
