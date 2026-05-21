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

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "camera_node.h"

namespace roborts_camera{
CameraNode::CameraNode(rclcpp::Node::SharedPtr node) : node_(node) {
  camera_num_ = camera_param_.GetCameraParam().size();
  img_pubs_.resize(camera_num_);
  camera_threads_.resize(camera_num_);
  camera_driver_.resize(camera_num_);

  for (unsigned int i = 0; i < camera_num_; i++) {
    auto camera_info = camera_param_.GetCameraParam()[i];
    image_transport::ImageTransport it(node_);
    img_pubs_[i] = it.advertiseCamera(camera_info.camera_name + "/image_raw", rclcpp::QoS(1));
    camera_driver_[i] = roborts_common::AlgorithmFactory<CameraBase,CameraInfo>::CreateAlgorithm(camera_info.camera_type,camera_info);
  }
  StartThread();
}

void CameraNode::StartThread() {
  running_ = true;
  for (unsigned int i = 0; i < camera_num_; i++) {
    camera_threads_[i] = std::thread(&CameraNode::Update, this, i);
  }
}

void CameraNode::Update(const unsigned int index) {
  cv::Mat img;
  while(running_) {
    camera_driver_[index]->StartReadCamera(img);
    if(!img.empty()) {
      auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
      img_msg->header.frame_id = camera_param_.GetCameraParam()[index].camera_name;
      img_msg->header.stamp = node_->now();
      camera_param_.GetCameraParam()[index].ros_camera_info->header.stamp = img_msg->header.stamp;
      img_pubs_[index].publish(img_msg, camera_param_.GetCameraParam()[index].ros_camera_info);
    }
  }
}

void CameraNode::StoptThread() {}

CameraNode::~CameraNode() {
  running_ = false;
  for (auto &iter: camera_threads_) {
    if (iter.joinable())
      iter.join();
  }
}
} //namespace roborts_camera

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("roborts_camera_node");
  roborts_camera::CameraNode camera_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
