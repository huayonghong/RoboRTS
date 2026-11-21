#ifndef ROBORTS_PERCEPTION__PERSON_TRACKER_HPP_
#define ROBORTS_PERCEPTION__PERSON_TRACKER_HPP_

#include <cstdint>
#include <vector>

#include "roborts_perception/pose_detector.hpp"

namespace roborts_perception
{

struct TrackedPerson
{
  uint32_t id{0};
  cv::Rect2f box;
  float score{0.0f};
  std::vector<cv::Point2f> keypoints;
  std::vector<float> keypoint_scores;
};

/// Greedy IoU tracker: keeps ids stable for a few missing frames so gestures
/// and gimbal following can refer to the same person.
class PersonTracker
{
public:
  PersonTracker(float iou_threshold = 0.3f, int max_missed = 30);

  std::vector<TrackedPerson> Update(
    const std::vector<PoseDetection> & detections);

private:
  struct Track
  {
    uint32_t id{0};
    cv::Rect2f box;
    int missed{0};
  };

  float iou_threshold_;
  int max_missed_;
  uint32_t next_id_{1};
  std::vector<Track> tracks_;
};

}  // namespace roborts_perception

#endif  // ROBORTS_PERCEPTION__PERSON_TRACKER_HPP_
