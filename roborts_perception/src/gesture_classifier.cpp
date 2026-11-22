#include "roborts_perception/gesture_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace roborts_perception
{

namespace
{

float Distance(const cv::Point2f & a, const cv::Point2f & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

class Hand
{
public:
  Hand(
    const std::vector<cv::Point2f> & points,
    const std::vector<float> & scores)
  : points_(points), scores_(scores)
  {
    scale_ = std::max(1.0f, Distance(points_[0], points_[9]));
    const int finger_pip[4] = {6, 10, 14, 18};
    const int finger_tip[4] = {8, 12, 16, 20};
    for (int finger = 0; finger < 4; ++finger) {
      fingers_[finger] = Extended(
        finger_pip[finger], finger_tip[finger], 1.20f);
    }
    // Thumb: compare the tip with the IP joint, and require it to be clear
    // of the index MCP so a tucked thumb is not counted as extended.
    thumb_ = Extended(3, 4, 1.15f) &&
      Distance(points_[4], points_[5]) > 0.5f * scale_ &&
      scores_[4] >= 0.5f;
  }

  bool Thumb() const {return thumb_;}
  bool Index() const {return fingers_[0];}
  bool Middle() const {return fingers_[1];}
  bool Ring() const {return fingers_[2];}
  bool Pinky() const {return fingers_[3];}
  // Open-hand detection is deliberately more permissive than pointing_up /
  // victory: an open palm with the thumb tucked still means "open hand", and
  // slight foreshortening must not demote it to pointing_up.
  bool IndexOpen() const {return Extended(6, 8, 1.10f);}
  bool MiddleOpen() const {return Extended(10, 12, 1.10f);}
  bool RingOpen() const {return Extended(14, 16, 1.10f);}
  bool PinkyOpen() const {return Extended(18, 20, 1.10f);}
  float Scale() const {return scale_;}
  const cv::Point2f & Point(int index) const {return points_[index];}
  float Score(int index) const {return scores_[index];}

  float MeanScore() const
  {
    return std::accumulate(scores_.begin(), scores_.end(), 0.0f) /
           static_cast<float>(scores_.size());
  }

private:
  bool Extended(int pip, int tip, float ratio) const
  {
    if (scores_[tip] < 0.5f || scores_[pip] < 0.5f) {
      return false;
    }
    return Distance(points_[0], points_[tip]) >
           ratio * Distance(points_[0], points_[pip]);
  }

  const std::vector<cv::Point2f> & points_;
  const std::vector<float> & scores_;
  float scale_{1.0f};
  bool thumb_{false};
  bool fingers_[4]{false, false, false, false};
};

}  // namespace

const char * GestureName(uint8_t id)
{
  switch (id) {
    case kGestureFist: return "fist";
    case kGesturePalm: return "palm";
    case kGestureVictory: return "victory";
    case kGesturePointingUp: return "pointing_up";
    case kGestureThumbsUp: return "thumbs_up";
    default: return "none";
  }
}

GestureResult ClassifyGesture(
  const std::vector<cv::Point2f> & keypoints,
  const std::vector<float> & keypoint_scores)
{
  GestureResult result;
  if (keypoints.size() < 21 || keypoint_scores.size() < 21) {
    return result;
  }
  const Hand hand(keypoints, keypoint_scores);
  const float mean_score = hand.MeanScore();
  const bool four_curled =
    !hand.Index() && !hand.Middle() && !hand.Ring() && !hand.Pinky();

  if (hand.IndexOpen() && hand.MiddleOpen() && hand.RingOpen() &&
    hand.PinkyOpen())
  {
    result = {kGesturePalm, GestureName(kGesturePalm), mean_score};
  } else if (hand.Index() && hand.Middle() && !hand.Ring() && !hand.Pinky()) {
    result = {kGestureVictory, GestureName(kGestureVictory), mean_score};
  } else if (hand.Index() && !hand.Middle() && !hand.Ring() && !hand.Pinky() &&
    hand.Point(8).y < hand.Point(6).y - 0.1f * hand.Scale())
  {
    result = {kGesturePointingUp, GestureName(kGesturePointingUp), mean_score};
  } else if (hand.Thumb() && four_curled &&
    hand.Point(4).y < hand.Point(0).y - 0.1f * hand.Scale())
  {
    result = {kGestureThumbsUp, GestureName(kGestureThumbsUp), mean_score};
  } else if (four_curled) {
    result = {kGestureFist, GestureName(kGestureFist), mean_score};
  }
  return result;
}

}  // namespace roborts_perception
