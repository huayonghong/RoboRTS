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

#ifndef ROBORTS_DETECTION_CVTOOLBOX_H
#define ROBORTS_DETECTION_CVTOOLBOX_H

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace roborts_detection {

enum BufferState {
  IDLE = 0,
  WRITE,
  READ
};

class CVToolbox {
 public:
  explicit CVToolbox(const std::string &camera_namespace,
                     unsigned int buffer_size = 3)
      : get_img_info_(false) {
    rclcpp::NodeOptions opts;
    node_ = std::make_shared<rclcpp::Node>(
        "cv_toolbox", camera_namespace[0] == '/' ? camera_namespace.substr(1) : camera_namespace,
        opts);
    logger_ = node_->get_logger();

    image_transport::ImageTransport it(node_);
    camera_sub_ = it.subscribeCamera(
        "image_raw",
        20,
        std::bind(&CVToolbox::ImageCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    image_buffer_.resize(buffer_size);
    buffer_state_.resize(buffer_size);
    index_ = 0;
    capture_time_ = -1;
    for (unsigned int i = 0; i < buffer_state_.size(); ++i) {
      buffer_state_[i] = BufferState::IDLE;
    }
    latest_index_ = -1;
  }

  rclcpp::Node::SharedPtr GetNode() const { return node_; }

  int GetCameraHeight() {
    if (!get_img_info_) {
      RCLCPP_WARN(logger_,
                  "Can not get camera height info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.height;
  }

  int GetCameraWidth() {
    if (!get_img_info_) {
      RCLCPP_WARN(logger_,
                  "Can not get camera width info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.width;
  }

  int GetCameraMatrix(cv::Mat &k_matrix) {
    if (!get_img_info_) {
      RCLCPP_WARN(logger_,
                  "Can not get camera matrix info, because the first frame data wasn't received");
      return -1;
    }
    k_matrix = cv::Mat(3, 3, CV_64F, reinterpret_cast<void *>(camera_info_.k.data())).clone();
    return 0;
  }

  int GetCameraDistortion(cv::Mat &distortion) {
    if (!get_img_info_) {
      RCLCPP_WARN(logger_,
                  "Can not get camera distortion info, because the first frame data wasn't received");
      return -1;
    }
    distortion = cv::Mat(camera_info_.d.size(), 1, CV_64F,
                         reinterpret_cast<void *>(camera_info_.d.data())).clone();
    return 0;
  }

  int GetWidthOffSet() {
    if (!get_img_info_) {
      RCLCPP_WARN(logger_,
                  "Can not get camera width offset info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.roi.x_offset;
  }

  int GetHeightOffSet() {
    if (!get_img_info_) {
      RCLCPP_WARN(logger_,
                  "Can not get camera height offset info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.roi.y_offset;
  }

  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg) {
    if (!get_img_info_) {
      camera_info_ = *camera_info_msg;
      capture_begin_ = std::chrono::high_resolution_clock::now();
      get_img_info_ = true;
    } else {
      capture_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>(
                          std::chrono::high_resolution_clock::now() - capture_begin_)
                          .count();
      capture_begin_ = std::chrono::high_resolution_clock::now();
    }
    capture_begin_ = std::chrono::high_resolution_clock::now();
    for (unsigned int i = 0; i < buffer_state_.size(); ++i) {
      if (buffer_state_[i] != BufferState::READ) {
        image_buffer_[i] = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
        buffer_state_[i] = BufferState::WRITE;
        lock_.lock();
        latest_index_ = static_cast<int>(i);
        lock_.unlock();
      }
    }
  }

  int NextImage(cv::Mat &src_img) {
    if (latest_index_ < 0) {
      RCLCPP_WARN(logger_, "Call image when no image received");
      return -1;
    }
    int temp_index = -1;
    lock_.lock();
    if (buffer_state_[latest_index_] == BufferState::WRITE) {
      buffer_state_[latest_index_] = BufferState::READ;
    } else {
      RCLCPP_INFO(logger_, "No image is available");
      lock_.unlock();
      return temp_index;
    }
    temp_index = latest_index_;
    lock_.unlock();

    src_img = image_buffer_[temp_index];
    return temp_index;
  }

  void GetCaptureTime(double &capture_time) {
    if (!get_img_info_) {
      RCLCPP_WARN(logger_, "The first image doesn't receive");
      return;
    }
    if (capture_time_ < 0) {
      RCLCPP_WARN(logger_, "The second image doesn't receive");
      return;
    }

    capture_time = capture_time_;
  }

  void ReadComplete(int return_index) {
    if (return_index < 0 || return_index > (static_cast<int>(buffer_state_.size()) - 1)) {
      RCLCPP_ERROR(logger_, "Return index error, please check the return_index");
      return;
    }

    buffer_state_[static_cast<size_t>(return_index)] = BufferState::IDLE;
  }

  cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv) {
    if (using_hsv) {
      cv::Mat img_hsv;
      cv::cvtColor(src_img, img_hsv, cv::COLOR_BGR2HSV);
      if (color == 0) {
        cv::Mat img_hsv_blue, img_threshold_blue;
        img_hsv_blue = img_hsv.clone();
        cv::Mat blue_low(cv::Scalar(90, 150, 46));
        cv::Mat blue_higher(cv::Scalar(140, 255, 255));
        cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
        return img_threshold_blue;
      } else {
        cv::Mat img_hsv_red1, img_hsv_red2, img_threshold_red, img_threshold_red1, img_threshold_red2;
        img_hsv_red1 = img_hsv.clone();
        img_hsv_red2 = img_hsv.clone();
        cv::Mat red1_low(cv::Scalar(0, 43, 46));
        cv::Mat red1_higher(cv::Scalar(3, 255, 255));

        cv::Mat red2_low(cv::Scalar(170, 43, 46));
        cv::Mat red2_higher(cv::Scalar(180, 255, 255));
        cv::inRange(img_hsv_red1, red1_low, red1_higher, img_threshold_red1);
        cv::inRange(img_hsv_red2, red2_low, red2_higher, img_threshold_red2);
        img_threshold_red = img_threshold_red1 | img_threshold_red2;
        return img_threshold_red;
      }
    } else {
      std::vector<cv::Mat> bgr;
      cv::split(src_img, bgr);
      if (color == 1) {
        cv::Mat result_img;
        cv::subtract(bgr[2], bgr[1], result_img);
        return result_img;
      } else if (color == 0) {
        cv::Mat result_img;
        cv::subtract(bgr[0], bgr[2], result_img);
        return result_img;
      }
    }
    return cv::Mat();
  }

  std::vector<std::vector<cv::Point>> FindContours(const cv::Mat &binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
  }

  void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color,
                       int thickness) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    float angle = rect.angle;
    std::ostringstream ss;
    ss << angle;
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

  void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color,
                       int thickness, float angle) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    std::ostringstream ss;
    ss << (int)(angle);
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 100, 0), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("cv_toolbox")};
  std::vector<cv::Mat> image_buffer_;
  std::vector<BufferState> buffer_state_;
  int latest_index_;
  std::mutex lock_;
  int index_;
  std::chrono::high_resolution_clock::time_point capture_begin_;
  double capture_time_;

  image_transport::CameraSubscriber camera_sub_;
  bool get_img_info_;
  sensor_msgs::msg::CameraInfo camera_info_;
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_CVTOOLBOX_H
