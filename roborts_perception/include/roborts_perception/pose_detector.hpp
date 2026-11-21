#ifndef ROBORTS_PERCEPTION__POSE_DETECTOR_HPP_
#define ROBORTS_PERCEPTION__POSE_DETECTOR_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "roborts_perception/trt_engine.hpp"

namespace roborts_perception
{

struct PoseDetection
{
  cv::Rect2f box;                       // original image pixels
  float score{0.0f};
  std::vector<cv::Point2f> keypoints;   // original image pixels
  std::vector<float> keypoint_scores;
};

struct PoseDecodeOptions
{
  float conf_threshold{0.25f};
  float iou_threshold{0.45f};
  int max_detections{30};
  int expected_keypoints{17};           // 17 body / 21 hand; 0 = auto
  float min_box_height_px{40.0f};       // measured in the 640 input space
  float keypoint_conf_threshold{0.3f};
  int min_valid_keypoints{3};
};

/// YOLO11-pose style detector: letterbox -> TensorRT -> decode (bbox +
/// keypoints) -> NMS, all in C++.
class PoseDetector
{
public:
  PoseDetector(
    const std::string & engine_path, const PoseDecodeOptions & options);

  std::vector<PoseDetection> Infer(const cv::Mat & bgr);

  int input_size() const {return input_size_;}
  int num_keypoints() const {return num_keypoints_;}
  double last_inference_ms() const {return engine_.last_inference_ms();}

private:
  struct Letterbox
  {
    float scale{1.0f};
    float pad_x{0.0f};
    float pad_y{0.0f};
  };

  Letterbox PrepareInput(const cv::Mat & bgr);
  std::vector<PoseDetection> Decode(
    const Letterbox & letterbox, const cv::Size & image_size) const;

  TrtEngine engine_;
  PoseDecodeOptions options_;
  int input_size_{0};
  int channels_{0};
  int anchors_{0};
  int num_keypoints_{0};
  bool channel_major_{true};
  std::vector<float> input_buffer_;
  std::vector<float> output_buffer_;
};

}  // namespace roborts_perception

#endif  // ROBORTS_PERCEPTION__POSE_DETECTOR_HPP_
