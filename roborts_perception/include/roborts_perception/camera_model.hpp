#ifndef ROBORTS_PERCEPTION__CAMERA_MODEL_HPP_
#define ROBORTS_PERCEPTION__CAMERA_MODEL_HPP_

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace roborts_perception
{

/// Pinhole camera intrinsics loaded from a ROS camera_info-style YAML file.
///
/// Pixel coordinates follow the image convention (u right, v down);
/// PixelToBearing() returns (yaw, pitch) in radians where yaw > 0 is to the
/// right of the optical axis and pitch > 0 is below it.
struct CameraModel
{
  int image_width{0};
  int image_height{0};
  cv::Matx33d camera_matrix{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  std::vector<double> distortion_coefficients;
  std::string distortion_model{"plumb_bob"};
  std::string camera_name{"csi_camera"};
  bool calibrated{true};
  std::optional<double> reprojection_error;

  double fx() const {return camera_matrix(0, 0);}
  double fy() const {return camera_matrix(1, 1);}
  double cx() const {return camera_matrix(0, 2);}
  double cy() const {return camera_matrix(1, 2);}

  bool HasDistortion() const;
  cv::Point2d PixelToBearing(double u, double v) const;
};

/// Load intrinsics from a camera_info-style YAML file.
/// Throws std::runtime_error if the file is missing or malformed.
CameraModel LoadCameraModel(const std::string & path);

}  // namespace roborts_perception

#endif  // ROBORTS_PERCEPTION__CAMERA_MODEL_HPP_
