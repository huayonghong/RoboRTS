#ifndef ROBORTS_PERCEPTION__GESTURE_CLASSIFIER_HPP_
#define ROBORTS_PERCEPTION__GESTURE_CLASSIFIER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace roborts_perception
{

enum GestureId : uint8_t
{
  kGestureNone = 0,
  kGestureFist = 1,
  kGesturePalm = 2,
  kGestureVictory = 3,
  kGesturePointingUp = 4,
  kGestureThumbsUp = 5,
};

struct GestureResult
{
  uint8_t id{kGestureNone};
  std::string name{"none"};
  float score{0.0f};
};

const char * GestureName(uint8_t id);

/// Rule-based classifier over the 21 MediaPipe-order hand landmarks:
/// 0 wrist, 1-4 thumb, 5-8 index, 9-12 middle, 13-16 ring, 17-20 pinky.
GestureResult ClassifyGesture(
  const std::vector<cv::Point2f> & keypoints,
  const std::vector<float> & keypoint_scores);

}  // namespace roborts_perception

#endif  // ROBORTS_PERCEPTION__GESTURE_CLASSIFIER_HPP_
