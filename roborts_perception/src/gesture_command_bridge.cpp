/// P7: turns confirmed gestures into safe chassis commands.
///
/// Gesture mapping (design doc 4.4):
///   palm         emergency stop (latches; re-enable through the service)
///   thumbs_up    forward  linear_speed
///   fist         backward backward_speed
///   victory      toggle visual following (/perception/follow_enable)
///   pointing_up  lock the gesture's track id (/perception/lock_target)
///
/// The bridge only publishes /cmd_vel_gesture while gesture mode is enabled
/// (or while the e-stop is latched); when disabled it stays silent so the mux
/// falls back to the planner.  If no hand/gesture is seen for hand_timeout it
/// publishes zero.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

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

int Priority(uint8_t id)
{
  switch (id) {
    case kGesturePalm: return 5;
    case kGesturePointingUp: return 4;
    case kGestureVictory: return 3;
    case kGestureThumbsUp: return 2;
    case kGestureFist: return 1;
    default: return 0;
  }
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
    backward_speed_ = declare_parameter<double>("backward_speed", 0.1);
    max_speed_ = declare_parameter<double>("max_speed", 0.2);
    hand_timeout_s_ = declare_parameter<double>("hand_timeout_s", 2.0);
    follow_enabled_ = declare_parameter<bool>("follow_enabled_on_start", true);
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
    if (publish_rate <= 0.0) {
      throw std::runtime_error("publish_rate must be positive");
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

    PublishFollowState();
    RCLCPP_INFO(
      get_logger(),
      "gesture control %s; call the ~/enable service to enter gesture mode",
      enabled_ ? "ENABLED" : "disabled");
  }

private:
  void OnGestures(const roborts_msgs::msg::GestureArray::SharedPtr message)
  {
    if (message->gestures.empty()) {
      current_gesture_ = kGestureNone;
      return;
    }
    last_gesture_seen_ = NowSeconds();

    const roborts_msgs::msg::Gesture * selected = nullptr;
    for (const auto & gesture : message->gestures) {
      if (selected == nullptr ||
        Priority(gesture.gesture_id) > Priority(selected->gesture_id))
      {
        selected = &gesture;
      }
    }
    if (selected == nullptr) {
      return;
    }
    const uint8_t id = selected->gesture_id;
    const uint8_t previous = current_gesture_;
    current_gesture_ = id;

    if (id == kGesturePalm) {
      if (!estop_latched_) {
        RCLCPP_WARN(get_logger(), "PALM: emergency stop latched");
      }
      estop_latched_ = true;
      enabled_ = false;
      command_ = geometry_msgs::msg::Twist();
      command_pub_->publish(command_);
      return;
    }

    if (!enabled_ || estop_latched_) {
      return;
    }

    switch (id) {
      case kGestureThumbsUp:
        command_.linear.x = linear_speed_;
        break;
      case kGestureFist:
        command_.linear.x = -backward_speed_;
        break;
      case kGestureVictory:
        command_ = geometry_msgs::msg::Twist();
        if (previous != kGestureVictory) {
          follow_enabled_ = !follow_enabled_;
          PublishFollowState();
          RCLCPP_INFO(
            get_logger(), "VICTORY: visual following %s",
            follow_enabled_ ? "on" : "off");
        }
        break;
      case kGesturePointingUp:
        command_ = geometry_msgs::msg::Twist();
        if (previous != kGesturePointingUp) {
          std_msgs::msg::UInt32 lock;
          lock.data = selected->track_id;
          lock_pub_->publish(lock);
          RCLCPP_INFO(
            get_logger(), "POINTING_UP: lock target %u", selected->track_id);
        }
        break;
      default:
        command_ = geometry_msgs::msg::Twist();
        break;
    }
    command_.linear.x = std::clamp(command_.linear.x, -max_speed_, max_speed_);
  }

  void Tick()
  {
    if (estop_latched_) {
      command_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }
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
      estop_latched_ = false;
      last_gesture_seen_ = NowSeconds();
      command_ = geometry_msgs::msg::Twist();
    } else {
      command_ = geometry_msgs::msg::Twist();
      command_pub_->publish(command_);
    }
    response->success = true;
    response->message = enabled_ ? "gesture control enabled" :
      "gesture control disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void PublishFollowState()
  {
    std_msgs::msg::Bool message;
    message.data = follow_enabled_;
    follow_pub_->publish(message);
  }

  bool enabled_{false};
  bool estop_latched_{false};
  bool follow_enabled_{true};
  double linear_speed_{0.15};
  double backward_speed_{0.1};
  double max_speed_{0.2};
  double hand_timeout_s_{2.0};
  double last_gesture_seen_{0.0};
  uint8_t current_gesture_{kGestureNone};
  geometry_msgs::msg::Twist command_;

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
