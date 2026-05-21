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

#include <functional>
#include <string>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

std::string topic_name = "back_camera";
cv::Mat src_img;

void ReceiveImg(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  src_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  static cv::VideoWriter writer(
      topic_name + ".avi",
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
      25.0,
      cv::Size(640, 360));
  writer.write(src_img);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("image_capture");

  image_transport::ImageTransport it(node);

  image_transport::Subscriber sub = it.subscribe(
      topic_name, rclcpp::QoS(20),
      std::bind(&ReceiveImg, std::placeholders::_1));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
