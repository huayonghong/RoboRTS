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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "localization_node.h"

#include <cmath>

#include <chrono>
#include <functional>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/create_timer_ros.hpp>

namespace roborts_localization {

namespace {

constexpr int kBlockingWaitSeconds = 600;

}  // namespace

bool LocalizationNode::FramesMatch(std::string a, std::string b) {
  if (!a.empty() && a.front() == '/') {
    a = a.substr(1);
  }
  if (!b.empty() && b.front() == '/') {
    b = b.substr(1);
  }
  return a == b;
}

bool LocalizationNode::SpinUntil(const std::function<bool()> &predicate) {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(kBlockingWaitSeconds);
  while (rclcpp::ok()) {
    if (predicate()) {
      return true;
    }
    exec.spin_some(std::chrono::milliseconds(50));
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
  }
  return false;
}

LocalizationNode::LocalizationNode(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), last_laser_msg_timestamp_(node_->now()) {
  CHECK(Init()) << "Module localization initialized failed!";
  initialized_ = true;
}

bool LocalizationNode::Init() {
  LocalizationConfig localization_config;

  localization_config.GetParam(node_);

  odom_frame_ = std::move(localization_config.odom_frame_id);
  global_frame_ = std::move(localization_config.global_frame_id);
  base_frame_ = std::move(localization_config.base_frame_id);

  laser_topic_ = std::move(localization_config.laser_topic_name);
  map_topic_ = std::move(localization_config.map_topic_name);
  auto init_pose_topic = std::move(localization_config.init_pose_topic_name);

  init_pose_ = {localization_config.initial_pose_x,
                localization_config.initial_pose_y,
                localization_config.initial_pose_a};
  init_cov_ = {localization_config.initial_cov_xx,
               localization_config.initial_cov_yy,
               localization_config.initial_cov_aa};

  transform_tolerance_ =
      rclcpp::Duration::from_seconds(localization_config.transform_tolerance);
  publish_visualize_ = localization_config.publish_visualize;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // ROS2: MessageFilter / waitForTransform need a timer interface on the buffer.
  tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface()));

  tf_listener_ptr_ =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_);

  tf_broadcaster_ptr_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  auto latched_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  distance_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "distance_map", latched_qos);
  particlecloud_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud",
                                                             latched_qos);

  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(2)));

  laser_scan_sub_ = std::make_shared<
      message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
      node_, laser_topic_,
      rclcpp::QoS(static_cast<size_t>(100)).get_rmw_qos_profile());

  laser_scan_filter_ =
      std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
          *laser_scan_sub_,
          *tf_buffer_,
          odom_frame_,
          100U,
          node_);

  laser_scan_filter_->registerCallback(std::bind(
      &LocalizationNode::LaserScanCallback, this, std::placeholders::_1));

  initial_pose_sub_ =
      node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          init_pose_topic, rclcpp::QoS(2),
          std::bind(&LocalizationNode::InitialPoseCallback, this,
                    std::placeholders::_1));

  amcl_ptr_ = std::make_unique<Amcl>();
  amcl_ptr_->GetParamFromRos(node_);
  amcl_ptr_->Init(init_pose_, init_cov_);

  map_init_ = WaitForStaticMap();
  laser_init_ = WaitForLaserPose();

  return map_init_ && laser_init_;
}

bool LocalizationNode::WaitForStaticMap() {
  nav_msgs::msg::OccupancyGrid map_msg;
  bool got = false;

  auto qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto sub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      qos,
      [&got, &map_msg](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        map_msg = *m;
        got = true;
      });

  if (!SpinUntil([&got]() { return got; })) {
    RCLCPP_ERROR(localization_logger(), "Timed out waiting for map on topic '%s'",
                 map_topic_.c_str());
    return false;
  }

  sub.reset();

  amcl_ptr_->HandleMapMessage(map_msg, init_pose_, init_cov_);
  RCLCPP_INFO(localization_logger(), "Received map on topic '%s'",
              map_topic_.c_str());

  return true;
}

bool LocalizationNode::WaitForLaserPose() {
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;
  bool got = false;
  auto sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic_,
      rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        laser_scan_msg = std::move(msg);

        got = true;
      });

  if (!SpinUntil([&got]() { return got; })) {
    RCLCPP_ERROR(localization_logger(),
                 "Timed out waiting for laser on topic '%s'", laser_topic_.c_str());

    sub.reset();

    return false;
  }

  sub.reset();

  Vec3d laser_pose;
  laser_pose.setZero();

  GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id,
                rclcpp::Time{0}, laser_pose);

  laser_pose[2] = 0;

  amcl_ptr_->SetLaserSensorPose(laser_pose);

  return true;
}

void LocalizationNode::InitialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
        &init_pose_msg) {

  if (init_pose_msg->header.frame_id.empty()) {
    RCLCPP_WARN(localization_logger(), "Received initial pose with empty frame_id.");

  }

  else if (!FramesMatch(init_pose_msg->header.frame_id, global_frame_)) {


    RCLCPP_ERROR(localization_logger(),
                 "Ignoring initial pose in frame \"%s\"; "
                 "initial poses must be in the global frame, \"%s\"",
                 init_pose_msg->header.frame_id.c_str(), global_frame_.c_str());

    return;
  }



  geometry_msgs::msg::TransformStamped tx_odom_msg;



  try {
    auto now = node_->now();



    tx_odom_msg =
        tf_buffer_->lookupTransform(base_frame_, init_pose_msg->header.stamp,
                                      base_frame_, now, odom_frame_,
                                      rclcpp::Duration::from_seconds(0.5));





  }



  catch (const tf2::TransformException &) {
    geometry_msgs::msg::TransformStamped identity;
    identity.transform.translation.x = 0.0;



    identity.transform.translation.y = 0.0;
    identity.transform.translation.z = 0.0;



    identity.transform.rotation.w = 1.0;



    tx_odom_msg = identity;







  }

  tf2::Transform pose_old;
  tf2::Transform pose_delta;
  tf2::Transform pose_new;
  tf2::fromMsg(init_pose_msg->pose.pose, pose_old);
  tf2::fromMsg(tx_odom_msg.transform, pose_delta);



  pose_new = pose_old * pose_delta;



  Vec3d init_pose_mean;
  Mat3d init_pose_cov;
  init_pose_mean.setZero();



  init_pose_cov.setZero();



  tf2Scalar roll{};



  tf2Scalar pitch{};
  tf2Scalar yaw{};







  tf2::Matrix3x3(pose_new.getRotation()).getRPY(roll, pitch, yaw);





  init_pose_mean(0) = pose_new.getOrigin().x();





  init_pose_mean(1) = pose_new.getOrigin().y();



  init_pose_mean(2) = static_cast<double>(yaw);







  init_pose_cov = math::MsgCovarianceToMat3d(init_pose_msg->pose.covariance);






  amcl_ptr_->HandleInitialPoseMessage(init_pose_mean, init_pose_cov);






}





void LocalizationNode::LaserScanCallback(




    const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser_scan_msg_ptr) {


  last_laser_msg_timestamp_ = rclcpp::Time(laser_scan_msg_ptr->header.stamp);





  Vec3d pose_in_odom;



  if (!GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_,



                     pose_in_odom)) {
    RCLCPP_ERROR(localization_logger(),




                 "Couldn't determine robot's pose");



    return;







  }






  double angle_min = 0, angle_increment = 0;
  sensor_msgs::msg::LaserScan laser_scan_msg = *laser_scan_msg_ptr;



  TransformLaserscanToBaseFrame(angle_min, angle_increment, laser_scan_msg);



  amcl_ptr_->Update(pose_in_odom,




                    laser_scan_msg,





                    angle_min,





                    angle_increment,








                    particlecloud_msg_,








                    hyp_pose_);







  if (!PublishTf()) {
    RCLCPP_ERROR(localization_logger(),




                 "Publish Tf Error!");

  }



  if (publish_visualize_) {
    PublishVisualize();





  }

}



void LocalizationNode::PublishVisualize() {


  if (pose_pub_->get_subscription_count() > 0) {
    pose_msg_.header.stamp = node_->now();



    pose_msg_.header.frame_id = global_frame_;



    pose_msg_.pose.position.x = hyp_pose_.pose_mean[0];





    pose_msg_.pose.position.y = hyp_pose_.pose_mean[1];

    pose_msg_.pose.position.z = 0.0;



    tf2::Quaternion q;





    q.setRPY(0, 0, hyp_pose_.pose_mean[2]);

    pose_msg_.pose.orientation = tf2::toMsg(q);



    pose_pub_->publish(pose_msg_);









  }









  if (particlecloud_pub_->get_subscription_count() > 0) {



    particlecloud_msg_.header.stamp = node_->now();






    particlecloud_msg_.header.frame_id = global_frame_;

    particlecloud_pub_->publish(particlecloud_msg_);

  }






  if (!publish_first_distance_map_) {






    distance_map_pub_->publish(amcl_ptr_->GetDistanceMapMsg());





    publish_first_distance_map_ = true;





  }





}







bool LocalizationNode::PublishTf() {





  auto transform_expiration =
      last_laser_msg_timestamp_ + transform_tolerance_;





  if (amcl_ptr_->CheckTfUpdate()) {






    geometry_msgs::msg::PoseStamped stamped_in;



    stamped_in.header.stamp = last_laser_msg_timestamp_;





    stamped_in.header.frame_id = base_frame_;

    tf2::Quaternion mq;



    mq.setRPY(0, 0, hyp_pose_.pose_mean[2]);





    tf2::Transform hyp_tf;



    hyp_tf =
        tf2::Transform(mq, tf2::Vector3(static_cast<float>(hyp_pose_.pose_mean[0]),




                                      static_cast<float>(hyp_pose_.pose_mean[1]),




                                      0.0));







    const tf2::Transform inv_tf = hyp_tf.inverse();
    stamped_in.pose.position.x = inv_tf.getOrigin().x();
    stamped_in.pose.position.y = inv_tf.getOrigin().y();
    stamped_in.pose.position.z = inv_tf.getOrigin().z();
    stamped_in.pose.orientation = tf2::toMsg(inv_tf.getRotation());







    geometry_msgs::msg::PoseStamped odom_pose;





    try {
      odom_pose =




          tf_buffer_->transform(stamped_in, odom_frame_, tf2::durationFromSec(0.05));


    }







    catch (const tf2::TransformException &) {


      RCLCPP_ERROR(localization_logger(),



                   "Failed to subtract base to odom transform");


      return false;











    }






    tf2::Quaternion q_odom;



    tf2::fromMsg(odom_pose.pose.orientation, q_odom);







    latest_tf_ = tf2::Transform(






        q_odom,






        tf2::Vector3(static_cast<float>(odom_pose.pose.position.x),




                     static_cast<float>(odom_pose.pose.position.y),





                     static_cast<float>(odom_pose.pose.position.z)));







    latest_tf_valid_ = true;



    geometry_msgs::msg::TransformStamped out_tf;



    out_tf.header.stamp = transform_expiration;



    out_tf.header.frame_id = global_frame_;



    out_tf.child_frame_id = odom_frame_;



    out_tf.transform = tf2::toMsg(latest_tf_.inverse());





    tf_broadcaster_ptr_->sendTransform(out_tf);



    sent_first_transform_ = true;


    return true;













  }






  if (latest_tf_valid_) {


    geometry_msgs::msg::TransformStamped tmp_tf;





    tmp_tf.header.stamp = transform_expiration;





    tmp_tf.header.frame_id = global_frame_;





    tmp_tf.child_frame_id = odom_frame_;





    tmp_tf.transform = tf2::toMsg(latest_tf_.inverse());





    tf_broadcaster_ptr_->sendTransform(tmp_tf);





    return true;



  }






  return false;



}









bool LocalizationNode::GetPoseFromTf(const std::string &target_frame,





                                     const std::string &source_frame,





                                     const rclcpp::Time &timestamp,





                                     Vec3d &pose) {





  geometry_msgs::msg::PoseStamped pose_in;





  pose_in.header.stamp = timestamp;



  pose_in.header.frame_id = source_frame;



  pose_in.pose.position.x = 0.0;



  pose_in.pose.position.y = 0.0;



  pose_in.pose.position.z = 0.0;



  pose_in.pose.orientation.w = 1.0;



  geometry_msgs::msg::PoseStamped pose_stamp;

  try {


    pose_stamp = tf_buffer_->transform(pose_in, target_frame,
                                       tf2::durationFromSec(0.05));







  }






  catch (const tf2::TransformException &) {


    return false;



  }








  pose.setZero();






  pose[0] = pose_stamp.pose.position.x;



  pose[1] = pose_stamp.pose.position.y;

  tf2::Quaternion qr;



  tf2::fromMsg(pose_stamp.pose.orientation, qr);



  tf2Scalar roll{};



  tf2Scalar pitch{};

  tf2Scalar yaw{};



  tf2::Matrix3x3(qr).getRPY(roll, pitch, yaw);



  pose[2] = static_cast<double>(yaw);



  return true;



}




void LocalizationNode::TransformLaserscanToBaseFrame(




    double &angle_min,





    double &angle_increment,





    const sensor_msgs::msg::LaserScan &laser_scan_msg) {








  geometry_msgs::msg::QuaternionStamped min_q;



  geometry_msgs::msg::QuaternionStamped inc_q;





  tf2::Quaternion q;



  q.setRPY(0.0,

           0.0,

           laser_scan_msg.angle_min);





  min_q.header = laser_scan_msg.header;





  min_q.quaternion = tf2::toMsg(q);





  q.setRPY(





      0.0,

      0.0,

      laser_scan_msg.angle_min +
          laser_scan_msg.angle_increment);



  inc_q.header = laser_scan_msg.header;



  inc_q.quaternion = tf2::toMsg(q);



  geometry_msgs::msg::QuaternionStamped min_base;



  geometry_msgs::msg::QuaternionStamped inc_base;



  try {






    min_base = tf_buffer_->transform(min_q,



                                       base_frame_, tf2::durationFromSec(0.5));


    inc_base = tf_buffer_->transform(





        inc_q, base_frame_, tf2::durationFromSec(0.5));


  }







  catch (const tf2::TransformException &e) {
    RCLCPP_WARN(localization_logger(),




                "Unable to transform min/max laser angles into base frame: %s",




                e.what());

    return;





  }








  tf2::Quaternion q_min;



  tf2::fromMsg(min_base.quaternion, q_min);





  angle_min = tf2::getYaw(q_min);





  tf2::Quaternion qi;





  tf2::fromMsg(inc_base.quaternion, qi);







  angle_increment = (tf2::getYaw(qi) - angle_min);







  angle_increment =




      (std::fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI);





}




}  // namespace roborts_localization





int main(int argc, char **argv) {


  roborts_localization::GLogWrapper glog_wrapper(argv[0]);



  rclcpp::init(argc, argv);



  auto node = std::make_shared<rclcpp::Node>("localization_node");



  roborts_localization::LocalizationNode localization_node(node);



  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), static_cast<size_t>(THREAD_NUM));



  executor.add_node(node);



  executor.spin();





  rclcpp::shutdown();





  return 0;



}
