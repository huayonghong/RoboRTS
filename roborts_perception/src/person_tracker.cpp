#include "roborts_perception/person_tracker.hpp"

#include <algorithm>
#include <vector>

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
  const float intersection =
    std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  const float union_area = a.width * a.height + b.width * b.height - intersection;
  return (union_area > 0.0f) ? intersection / union_area : 0.0f;
}

}  // namespace

PersonTracker::PersonTracker(float iou_threshold, int max_missed)
: iou_threshold_(iou_threshold), max_missed_(max_missed)
{
}

std::vector<TrackedPerson> PersonTracker::Update(
  const std::vector<PoseDetection> & detections)
{
  std::vector<TrackedPerson> result;
  std::vector<char> track_matched(tracks_.size(), 0);
  result.reserve(detections.size());

  for (const PoseDetection & detection : detections) {
    int best_track = -1;
    float best_iou = iou_threshold_;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (track_matched[i]) {
        continue;
      }
      const float iou = IoU(tracks_[i].box, detection.box);
      if (iou > best_iou) {
        best_iou = iou;
        best_track = static_cast<int>(i);
      }
    }

    TrackedPerson person;
    person.box = detection.box;
    person.score = detection.score;
    person.keypoints = detection.keypoints;
    person.keypoint_scores = detection.keypoint_scores;

    if (best_track >= 0) {
      Track & track = tracks_[best_track];
      track.box = detection.box;
      track.missed = 0;
      track_matched[best_track] = 1;
      person.id = track.id;
    } else {
      Track track;
      track.id = next_id_++;
      track.box = detection.box;
      track.missed = 0;
      tracks_.push_back(track);
      track_matched.push_back(1);
      person.id = track.id;
    }
    result.push_back(std::move(person));
  }

  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (!track_matched[i]) {
      ++tracks_[i].missed;
    }
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & track) {return track.missed > max_missed_;}),
    tracks_.end());

  return result;
}

}  // namespace roborts_perception
