#include "roborts_perception/camera_model.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

namespace roborts_perception
{

bool CameraModel::HasDistortion() const
{
  for (const double coefficient : distortion_coefficients) {
    if (std::abs(coefficient) > 1e-12) {
      return true;
    }
  }
  return false;
}

cv::Point2d CameraModel::PixelToBearing(double u, double v) const
{
  double x = 0.0;
  double y = 0.0;
  if (HasDistortion()) {
    std::vector<cv::Point2f> source{cv::Point2f(
        static_cast<float>(u), static_cast<float>(v))};
    std::vector<cv::Point2f> normalized;
    cv::undistortPoints(
      source, normalized, cv::Mat(camera_matrix),
      cv::Mat(distortion_coefficients));
    x = normalized[0].x;
    y = normalized[0].y;
  } else {
    x = (u - cx()) / fx();
    y = (v - cy()) / fy();
  }
  return cv::Point2d(std::atan2(x, 1.0), std::atan2(y, 1.0));
}

CameraModel LoadCameraModel(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "cannot load camera intrinsics from " + path + ": " + error.what());
  }

  CameraModel model;
  try {
    model.image_width = root["image_width"].as<int>();
    model.image_height = root["image_height"].as<int>();

    const YAML::Node matrix = root["camera_matrix"]["data"];
    if (!matrix || !matrix.IsSequence()) {
      throw std::runtime_error("camera_matrix.data must be a sequence");
    }
    const std::vector<double> values = matrix.as<std::vector<double>>();
    if (values.size() != 9) {
      throw std::runtime_error("camera_matrix.data must have 9 entries");
    }
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        model.camera_matrix(row, column) = values[row * 3 + column];
      }
    }

    const YAML::Node distortion = root["distortion_coefficients"]["data"];
    if (distortion && distortion.IsSequence()) {
      model.distortion_coefficients = distortion.as<std::vector<double>>();
    }
    if (root["distortion_model"] && !root["distortion_model"].IsNull()) {
      model.distortion_model = root["distortion_model"].as<std::string>();
    }
    if (root["camera_name"] && !root["camera_name"].IsNull()) {
      model.camera_name = root["camera_name"].as<std::string>();
    }
    if (root["calibrated"] && !root["calibrated"].IsNull()) {
      model.calibrated = root["calibrated"].as<bool>();
    }
    if (root["reprojection_error"] && !root["reprojection_error"].IsNull() &&
      root["reprojection_error"].IsScalar())
    {
      model.reprojection_error = root["reprojection_error"].as<double>();
    }
  } catch (const std::exception & error) {
    throw std::runtime_error(
            path + ": invalid camera intrinsics: " + error.what());
  }

  if (!std::isfinite(model.fx()) || !std::isfinite(model.fy()) ||
    model.fx() <= 0.0 || model.fy() <= 0.0)
  {
    throw std::runtime_error(
            path + ": camera_matrix is invalid (fx and fy must be positive)");
  }
  return model;
}

}  // namespace roborts_perception
