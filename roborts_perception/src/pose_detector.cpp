#include "roborts_perception/pose_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace roborts_perception
{

namespace
{

float IoU(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float x1 = std::max(a.x, b.x);
  const float y1 = std::max(a.y, b.y);
  const float x2 = std::min(a.x + a.width, b.x + b.width);
  const float y2 = std::min(a.y + a.height, b.y + b.height);
  const float inter_w = std::max(0.0f, x2 - x1);
  const float inter_h = std::max(0.0f, y2 - y1);
  const float intersection = inter_w * inter_h;
  const float area_a = a.width * a.height;
  const float area_b = b.width * b.height;
  const float union_area = area_a + area_b - intersection;
  return (union_area > 0.0f) ? intersection / union_area : 0.0f;
}

}  // namespace

PoseDetector::PoseDetector(
  const std::string & engine_path, const PoseDecodeOptions & options)
: engine_(engine_path), options_(options)
{
  const nvinfer1::Dims input_dims = engine_.input_info().dims;
  if (input_dims.nbDims != 4) {
    throw std::runtime_error("pose engine input must be NCHW (4 dims)");
  }
  input_size_ = input_dims.d[2];
  if (input_dims.d[1] != 3 || input_dims.d[3] != input_size_) {
    throw std::runtime_error("pose engine input must be 1x3xSxS");
  }

  const nvinfer1::Dims output_dims = engine_.output_info().dims;
  if (output_dims.nbDims != 3) {
    throw std::runtime_error(
            "pose engine output must be (1, channels, anchors)");
  }
  const int d1 = output_dims.d[1];
  const int d2 = output_dims.d[2];
  const int expected = options_.expected_keypoints;
  const auto matches = [expected](int dim) {
      if (dim < 5 || (dim - 5) % 3 != 0) {
        return false;
      }
      return expected <= 0 || (dim - 5) / 3 == expected;
    };

  if (matches(d1)) {
    channels_ = d1;
    anchors_ = d2;
    channel_major_ = true;
  } else if (matches(d2)) {
    channels_ = d2;
    anchors_ = d1;
    channel_major_ = false;
  } else {
    throw std::runtime_error(
            "pose engine output layout is not 5+3*K per anchor");
  }
  num_keypoints_ = (channels_ - 5) / 3;
  if (expected > 0 && num_keypoints_ != expected) {
    throw std::runtime_error("pose engine keypoint count does not match");
  }

  input_buffer_.assign(
    static_cast<size_t>(3) * input_size_ * input_size_, 114.0f / 255.0f);
  output_buffer_.resize(engine_.output_info().element_count);
}

PoseDetector::Letterbox PoseDetector::PrepareInput(const cv::Mat & bgr)
{
  Letterbox letterbox;
  const float scale = std::min(
    static_cast<float>(input_size_) / bgr.cols,
    static_cast<float>(input_size_) / bgr.rows);
  const int new_width = static_cast<int>(std::lround(bgr.cols * scale));
  const int new_height = static_cast<int>(std::lround(bgr.rows * scale));
  const int pad_x = static_cast<int>(
    std::lround((input_size_ - new_width) / 2.0));
  const int pad_y = static_cast<int>(
    std::lround((input_size_ - new_height) / 2.0));
  letterbox.scale = scale;
  letterbox.pad_x = static_cast<float>(pad_x);
  letterbox.pad_y = static_cast<float>(pad_y);

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(new_width, new_height));

  std::fill(input_buffer_.begin(), input_buffer_.end(), 114.0f / 255.0f);
  const size_t plane = static_cast<size_t>(input_size_) * input_size_;
  for (int y = 0; y < new_height; ++y) {
    const cv::Vec3b * row = resized.ptr<cv::Vec3b>(y);
    for (int x = 0; x < new_width; ++x) {
      const size_t offset =
        static_cast<size_t>(y + pad_y) * input_size_ + (x + pad_x);
      input_buffer_[offset] = row[x][2] / 255.0f;            // R
      input_buffer_[plane + offset] = row[x][1] / 255.0f;    // G
      input_buffer_[2 * plane + offset] = row[x][0] / 255.0f;  // B
    }
  }
  return letterbox;
}

std::vector<PoseDetection> PoseDetector::Infer(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return {};
  }
  const Letterbox letterbox = PrepareInput(bgr);
  engine_.Infer(input_buffer_, &output_buffer_);
  return Decode(letterbox, bgr.size());
}

std::vector<PoseDetection> PoseDetector::Decode(
  const Letterbox & letterbox, const cv::Size & image_size) const
{
  const float * data = output_buffer_.data();
  const auto value = [this, data](int channel, int anchor) {
      return channel_major_ ?
             data[static_cast<size_t>(channel) * anchors_ + anchor] :
             data[static_cast<size_t>(anchor) * channels_ + channel];
    };

  std::vector<PoseDetection> candidates;
  candidates.reserve(16);
  for (int anchor = 0; anchor < anchors_; ++anchor) {
    const float score = value(4, anchor);
    if (score < options_.conf_threshold) {
      continue;
    }
    const float width = value(2, anchor);
    const float height = value(3, anchor);
    if (width <= 0.0f || height < options_.min_box_height_px) {
      continue;
    }

    PoseDetection detection;
    detection.score = score;
    const float cx = value(0, anchor);
    const float cy = value(1, anchor);
    float x1 = (cx - width * 0.5f - letterbox.pad_x) / letterbox.scale;
    float y1 = (cy - height * 0.5f - letterbox.pad_y) / letterbox.scale;
    float x2 = (cx + width * 0.5f - letterbox.pad_x) / letterbox.scale;
    float y2 = (cy + height * 0.5f - letterbox.pad_y) / letterbox.scale;
    x1 = std::max(0.0f, std::min(x1, static_cast<float>(image_size.width - 1)));
    y1 = std::max(0.0f, std::min(y1, static_cast<float>(image_size.height - 1)));
    x2 = std::max(0.0f, std::min(x2, static_cast<float>(image_size.width - 1)));
    y2 = std::max(0.0f, std::min(y2, static_cast<float>(image_size.height - 1)));
    if (x2 - x1 < 2.0f || y2 - y1 < 2.0f) {
      continue;
    }
    detection.box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);

    detection.keypoints.resize(num_keypoints_);
    detection.keypoint_scores.resize(num_keypoints_);
    int valid_keypoints = 0;
    for (int k = 0; k < num_keypoints_; ++k) {
      const int base = 5 + 3 * k;
      const float kx = (value(base, anchor) - letterbox.pad_x) / letterbox.scale;
      const float ky = (value(base + 1, anchor) - letterbox.pad_y) / letterbox.scale;
      const float confidence = value(base + 2, anchor);
      detection.keypoints[k] = cv::Point2f(kx, ky);
      detection.keypoint_scores[k] = confidence;
      if (confidence >= options_.keypoint_conf_threshold &&
        kx >= 0.0f && kx < image_size.width &&
        ky >= 0.0f && ky < image_size.height)
      {
        ++valid_keypoints;
      }
    }
    if (valid_keypoints < options_.min_valid_keypoints) {
      continue;
    }
    candidates.push_back(std::move(detection));
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const PoseDetection & a, const PoseDetection & b) {
      return a.score > b.score;
    });

  std::vector<PoseDetection> result;
  std::vector<char> suppressed(candidates.size(), 0);
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (suppressed[i]) {
      continue;
    }
    result.push_back(candidates[i]);
    if (static_cast<int>(result.size()) >= options_.max_detections) {
      break;
    }
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (!suppressed[j] &&
        IoU(candidates[i].box, candidates[j].box) > options_.iou_threshold)
      {
        suppressed[j] = 1;
      }
    }
  }
  return result;
}

}  // namespace roborts_perception
