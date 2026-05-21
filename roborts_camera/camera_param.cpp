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

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "camera_param.h"
#include "camera_param.pb.h"
#include "io/io.h"

namespace roborts_camera{

CameraParam::CameraParam() {
  LoadCameraParam();
}

void CameraParam::LoadCameraParam() {
  Cameras camera_info;
  std::string file_name =
    ament_index_cpp::get_package_share_directory("roborts_camera") + "/config/camera_param.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &camera_info);
  if (!read_state) {
    RCLCPP_FATAL(rclcpp::get_logger("roborts_camera::CameraParam"),
                 "Cannot open %s", file_name.c_str());
    throw std::runtime_error(std::string("Cannot open ") + file_name);
  }

  int camera_num = camera_info.camera().size();
  cameras_param_.resize(camera_num);
  for(unsigned int index = 0; index < camera_num; index++) {

    cameras_param_[index].camera_name = camera_info.camera(index).camera_name();
    cameras_param_[index].camera_type = camera_info.camera(index).camera_type();
    cameras_param_[index].camera_path = camera_info.camera(index).camera_path();

    cameras_param_[index].resolution_width = camera_info.camera(index).resolution().width();
    cameras_param_[index].resolution_height = camera_info.camera(index).resolution().height();
    cameras_param_[index].width_offset = camera_info.camera(index).resolution().width_offset();
    cameras_param_[index].height_offset = camera_info.camera(index).resolution().height_offset();

    cameras_param_[index].fps = camera_info.camera(index).fps();

    cameras_param_[index].auto_exposure = camera_info.camera(index).auto_exposure();
    cameras_param_[index].exposure_value = camera_info.camera(index).exposure_value();
    cameras_param_[index].exposure_time = camera_info.camera(index).exposure_time();

    cameras_param_[index].auto_white_balance = camera_info.camera(index).auto_white_balance();
    cameras_param_[index].auto_gain = camera_info.camera(index).auto_gain();

    cameras_param_[index].contrast = camera_info.camera(index).contrast();

    int camera_m_size = camera_info.camera(index).camera_matrix().data().size();
    double camera_m[camera_m_size];
    std::copy(camera_info.camera(index).camera_matrix().data().begin(),
              camera_info.camera(index).camera_matrix().data().end(),
              camera_m);
    cameras_param_[index].camera_matrix = cv::Mat(3, 3, CV_64F, camera_m).clone();

    int rows = camera_info.camera(index).camera_distortion().data_size();
    double camera_dis[rows];
    std::copy(camera_info.camera(index).camera_distortion().data().begin(),
              camera_info.camera(index).camera_distortion().data().end(),
              camera_dis);
    cameras_param_[index].camera_distortion = cv::Mat(rows, 1, CV_64F, camera_dis).clone();

    cameras_param_[index].ros_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    auto & ros_info = cameras_param_[index].ros_camera_info;
    ros_info->header.frame_id = cameras_param_[index].camera_name;
    ros_info->width = cameras_param_[index].resolution_width;
    ros_info->height = cameras_param_[index].resolution_height;
    ros_info->distortion_model = "plumb_bob";

    ros_info->d.resize(static_cast<size_t>(rows));
    for (int r = 0; r < rows; r++) {
      ros_info->d[static_cast<size_t>(r)] = cameras_param_[index].camera_distortion.at<double>(r, 0);
    }

    ros_info->roi.x_offset = cameras_param_[index].width_offset;
    ros_info->roi.y_offset = cameras_param_[index].height_offset;

    ros_info->k.fill(0.0);
    {
      const int copy_n = std::min(camera_m_size, static_cast<int>(ros_info->k.size()));
      std::memcpy(ros_info->k.data(), camera_m, static_cast<size_t>(copy_n) * sizeof(double));
    }

    ros_info->r = {{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0}};

    const cv::Mat & Kmat = cameras_param_[index].camera_matrix;
    ros_info->p = {{
      Kmat.at<double>(0,0), 0.0, Kmat.at<double>(0,2), 0.0,
      0.0, Kmat.at<double>(1,1), Kmat.at<double>(1,2), 0.0,
      0.0, 0.0, Kmat.at<double>(2,2), 0.0}};
  }
}

void CameraParam::GetCameraParam(std::vector<CameraInfo> &cameras_param) {
  cameras_param = cameras_param_;
}

std::vector<CameraInfo>& CameraParam::GetCameraParam() {
  return cameras_param_;
}

} //namespace roborts_camera
