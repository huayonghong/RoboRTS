/// P7: turns confirmed gestures into target tracking and safe chassis commands.
///
/// Tracking switch (edge-triggered, per tracked person):
///   open hand -> fist   lock that person's track_id and enable visual
///                       following
///   fist -> open hand   release the lock and disable visual following
///
/// "Open hand" is palm, victory or pointing_up: this hand-pose model often
/// labels a loosely opened hand as pointing_up/victory, and requiring an exact
/// palm made the switch unreliable in practice.
///
/// The two gestures must belong to the same tracking id and arrive within
/// transition_timeout_s; short detection gaps between them are tolerated
/// (the fist needs gesture_min_frames to confirm, and the gesture node's
/// cooldown can briefly hide it).  Tracking transitions are honoured whenever
/// this node runs, because they only move the gimbal.
///
/// Motion (gesture mode must be enabled through ~/enable first):
///   thumbs_up      forward linear_speed
///
/// The bridge only publishes /cmd_vel_gesture while gesture mode is enabled;
/// when disabled it stays silent so the mux falls back to the planner.  If no
/// hand/gesture is seen for hand_timeout it publishes zero.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <roborts_msgs/msg/gesture_array.hpp>

#include "roborts_perception/gesture_classifier.hpp"

namespace roborts_perception
{

namespace
{

double NowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// The hand-pose model on this robot often labels a loosely opened hand as
/// pointing_up or victory rather than palm.  All three mean "hand open" for
/// the tracking switch; only a fist means "closed".
bool IsOpenHandGesture(uint8_t id)
{
  return id == kGesturePalm || id == kGestureVictory ||
         id == kGesturePointingUp;
}

}  // namespace

class GestureCommandBridge : public rclcpp::Node
{
public:
  GestureCommandBridge()
  : rclcpp::Node("gesture_command_bridge")
  {
    enabled_ = declare_parameter<bool>("enabled", false);
    linear_speed_ = declare_parameter<double>("linear_speed", 0.15);
    max_speed_ = declare_parameter<double>("max_speed", 0.2);
    hand_timeout_s_ = declare_parameter<double>("hand_timeout_s", 2.0);
    transition_timeout_s_ =
      declare_parameter<double>("transition_timeout_s", 1.5);
    follow_enabled_ = declare_parameter<bool>("follow_enabled_on_start", false);
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
    if (publish_rate <= 0.0) {
      throw std::runtime_error("publish_rate must be positive");
    }
    if (transition_timeout_s_ <= 0.0) {
      throw std::runtime_error("transition_timeout_s must be positive");
    }
    if (max_speed_ > 0.2) {
      RCLCPP_WARN(
        get_logger(), "max_speed=%.2f exceeds the 0.2 m/s gesture limit; "
        "clamping", max_speed_);
      max_speed_ = 0.2;
    }

    gesture_sub_ = create_subscription<roborts_msgs::msg::GestureArray>(
      "/perception/gestures", rclcpp::QoS(10),
      std::bind(&GestureCommandBridge::OnGestures, this, std::placeholders::_1));
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_gesture", rclcpp::QoS(10));
    follow_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/perception/follow_enable", rclcpp::QoS(1));
    lock_pub_ = create_publisher<std_msgs::msg::UInt32>(
      "/perception/lock_target", rclcpp::QoS(1));
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable",
      std::bind(
        &GestureCommandBridge::OnEnable, this, std::placeholders::_1,
        std::placeholders::_2));

    last_gesture_seen_ = NowSeconds();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = create_wall_timer(
      period, std::bind(&GestureCommandBridge::Tick, this));

    // Start idle: no target, following off.  The palm -> fist edge arms it.
    PublishLock(0);
    PublishFollowState();
    RCLCPP_INFO(
      get_logger(),
      "tracking idle; open hand -> fist locks the person and starts "
      "following, fist -> open hand stops.  Motion gestures are %s%s",
      enabled_ ? "ENABLED" : "disabled",
      enabled_ ? "" : " (call the ~/enable service to allow them)");
  }

private:
  struct TrackGesture
  {
    uint8_t id{kGestureNone};
    double seen{0.0};
  };

  void OnGestures(const roborts_msgs::msg::GestureArray::SharedPtr message)
  {
    const double now = NowSeconds();
    if (message->gestures.empty()) {
      PruneTrackGestures(now);
      return;
    }
    last_gesture_seen_ = now;

    bool forward = false;
    for (const auto & gesture : message->gestures) {
      if (gesture.gesture_id == kGestureNone || gesture.track_id == 0) {
        continue;
      }
      TrackGesture & state = track_gestures_[gesture.track_id];
      const bool previous_is_fresh =
        state.id != kGestureNone &&
        (now - state.seen) <= transition_timeout_s_;

      if (previous_is_fresh &&
        IsOpenHandGesture(state.id) && gesture.gesture_id == kGestureFist)
      {
        StartTracking(gesture.track_id);
      } else if (previous_is_fresh &&
        state.id == kGestureFist && IsOpenHandGesture(gesture.gesture_id))
      {
        StopTracking();
      }

      if (!tracking_ && IsOpenHandGesture(gesture.gesture_id) &&
        gesture.gesture_id != state.id)
      {
        RCLCPP_INFO(
          get_logger(),
          "track %u: open hand (%s) seen; close into a fist to start "
          "following", gesture.track_id, GestureName(gesture.gesture_id));
      }

      state.id = gesture.gesture_id;
      state.seen = now;

      if (gesture.gesture_id == kGestureThumbsUp) {
        forward = true;
      }
    }
    PruneTrackGestures(now);

    // Only the explicit forward gesture moves the chassis; palm/fist are the
    // tracking switch and are intentionally stationary.
    if (enabled_) {
      command_.linear.x = forward ? linear_speed_ : 0.0;
      command_.linear.x = std::clamp(command_.linear.x, -max_speed_, max_speed_);
    }
  }

  void StartTracking(uint32_t track_id)
  {
    if (tracking_ && tracked_id_ == track_id) {
      return;
    }
    const bool switched = tracking_ && tracked_id_ != track_id;
    tracking_ = true;
    tracked_id_ = track_id;
    // Lock first, then enable following, so the first follow frame already
    // sees the intended target.
    PublishLock(track_id);
    SetFollow(true);
    RCLCPP_INFO(
      get_logger(), "OPEN -> FIST: %s person %u (tracking on)",
      switched ? "switched lock to" : "locked", track_id);
  }

  void StopTracking()
  {
    if (!tracking_ && !follow_enabled_) {
      return;
    }
    const uint32_t released_id = tracked_id_;
    tracking_ = false;
    tracked_id_ = 0;
    // Disable following before dropping the lock so the follower never falls
    // back to another person for a frame.
    SetFollow(false);
    PublishLock(0);
    command_ = geometry_msgs::msg::Twist();
    RCLCPP_INFO(
      get_logger(), "FIST -> OPEN: tracking stopped (released person %u)",
      released_id);
  }

  void PruneTrackGestures(double now)
  {
    for (auto it = track_gestures_.begin(); it != track_gestures_.end();) {
      if (now - it->second.seen > transition_timeout_s_) {
        it = track_gestures_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void Tick()
  {
    if (!enabled_) {
      return;
    }
    if (NowSeconds() - last_gesture_seen_ > hand_timeout_s_) {
      command_ = geometry_msgs::msg::Twist();
    }
    command_.linear.y = 0.0;
    command_.linear.z = 0.0;
    command_.angular.x = 0.0;
    command_.angular.y = 0.0;
    command_.angular.z = 0.0;
    command_pub_->publish(command_);
  }

  void OnEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    enabled_ = request->data;
    if (enabled_) {
      last_gesture_seen_ = NowSeconds();
    }
    command_ = geometry_msgs::msg::Twist();
    if (!enabled_) {
      command_pub_->publish(command_);
    }
    response->success = true;
    response->message = enabled_ ? "gesture motion enabled" :
      "gesture motion disabled (tracking switch stays active)";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void PublishLock(uint32_t track_id)
  {
    std_msgs::msg::UInt32 message;
    message.data = track_id;
    lock_pub_->publish(message);
  }

  void SetFollow(bool enabled)
  {
    if (follow_enabled_ == enabled) {
      return;
    }
    follow_enabled_ = enabled;
    PublishFollowState();
  }

  void PublishFollowState()
  {
    std_msgs::msg::Bool message;
    message.data = follow_enabled_;
    follow_pub_->publish(message);
  }

  bool enabled_{false};
  bool tracking_{false};
  uint32_t tracked_id_{0};
  bool follow_enabled_{false};
  double linear_speed_{0.15};
  double max_speed_{0.2};
  double hand_timeout_s_{2.0};
  double transition_timeout_s_{1.5};
  double last_gesture_seen_{0.0};
  geometry_msgs::msg::Twist command_;
  std::unordered_map<uint32_t, TrackGesture> track_gestures_;

  rclcpp::Subscription<roborts_msgs::msg::GestureArray>::SharedPtr gesture_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr lock_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roborts_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<roborts_perception::GestureCommandBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("gesture_command_bridge"), "fatal: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
