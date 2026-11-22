/// P6: visual gimbal following.
///
/// Consumes /perception/persons, converts the locked target's pixel position
/// (via its yaw/pitch from the camera model) into an absolute gimbal angle and
/// publishes /cmd_gimbal_angle.  Deadband, slew limit, hold, return-to-centre
/// and optional yaw clamping are all configurable.
///
/// Following starts disabled: gesture_command_bridge enables it together with
/// a target lock on the palm -> fist edge, and disables it on fist -> palm.
/// Set follow_enabled:=true only if the gimbal should follow the best-scoring
/// person from start-up without a lock.
///
/// The only sign/zero assumptions left for the bench are yaw_sign/pitch_sign:
/// default -1 matches "gimbal yaw/pitch increase counter-clockwise/up", with
/// the camera bearing yaw>0 right and pitch>0 down.  Verify on the robot before
/// enabling autonomous following.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <roborts_msgs/msg/gimbal_angle.hpp>
#include <roborts_msgs/msg/person_array.hpp>
#include <roborts_msgs/srv/gimbal_mode.hpp>

namespace roborts_perception
{

namespace
{

constexpr double kDegToRad = M_PI / 180.0;

double NowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool IsFinite(double value)
{
  return std::isfinite(value);
}

}  // namespace

class GimbalFollowNode : public rclcpp::Node
{
public:
  GimbalFollowNode()
  : rclcpp::Node("gimbal_follow_node"), tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal");
    follow_enabled_ = declare_parameter<bool>("follow_enabled", false);

    yaw_gain_ = declare_parameter<double>("yaw_gain", 0.9);
    pitch_gain_ = declare_parameter<double>("pitch_gain", 0.9);
    yaw_sign_ = declare_parameter<double>("yaw_sign", -1.0);
    pitch_sign_ = declare_parameter<double>("pitch_sign", -1.0);
    deadband_rad_ = declare_parameter<double>("deadband_deg", 0.8) * kDegToRad;
    max_rate_rad_s_ =
      declare_parameter<double>("max_rate_deg_s", 180.0) * kDegToRad;
    hold_timeout_s_ = declare_parameter<double>("hold_timeout_s", 0.5);
    return_center_time_s_ =
      declare_parameter<double>("return_center_time_s", 3.0);
    center_yaw_rad_ =
      declare_parameter<double>("return_center_yaw_deg", 0.0) * kDegToRad;
    center_pitch_rad_ =
      declare_parameter<double>("return_center_pitch_deg", 0.0) * kDegToRad;
    allow_chassis_follow_ =
      declare_parameter<bool>("allow_chassis_follow", false);
    yaw_min_rad_ = declare_parameter<double>("yaw_min_deg", -90.0) * kDegToRad;
    yaw_max_rad_ = declare_parameter<double>("yaw_max_deg", 90.0) * kDegToRad;
    pitch_min_rad_ =
      declare_parameter<double>("pitch_min_deg", -30.0) * kDegToRad;
    pitch_max_rad_ =
      declare_parameter<double>("pitch_max_deg", 20.0) * kDegToRad;
    const bool set_control_mode =
      declare_parameter<bool>("set_control_mode", true);
    const int control_mode = declare_parameter<int>("control_mode", 0);
    const double publish_rate = declare_parameter<double>("publish_rate", 50.0);

    if (publish_rate <= 0.0) {
      throw std::runtime_error("publish_rate must be positive");
    }
    if (yaw_min_rad_ >= yaw_max_rad_ || pitch_min_rad_ >= pitch_max_rad_) {
      throw std::runtime_error("gimbal soft limits are invalid");
    }

    persons_sub_ = create_subscription<roborts_msgs::msg::PersonArray>(
      "/perception/persons", rclcpp::QoS(10),
      std::bind(&GimbalFollowNode::OnPersons, this, std::placeholders::_1));
    lock_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/perception/lock_target", rclcpp::QoS(1),
      std::bind(&GimbalFollowNode::OnLockTarget, this, std::placeholders::_1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/perception/follow_enable", rclcpp::QoS(1),
      std::bind(&GimbalFollowNode::OnFollowEnable, this, std::placeholders::_1));

    gimbal_pub_ = create_publisher<roborts_msgs::msg::GimbalAngle>(
      "/cmd_gimbal_angle", rclcpp::QoS(1));

    if (set_control_mode) {
      mode_client_ = create_client<roborts_msgs::srv::GimbalMode>(
        "/set_gimbal_mode");
      control_mode_ = control_mode;
      mode_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GimbalFollowNode::RequestControlMode, this));
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    control_timer_ = create_wall_timer(
      period, std::bind(&GimbalFollowNode::ControlLoop, this));

    last_target_seen_ = NowSeconds();
    last_control_time_ = NowSeconds();
    RCLCPP_INFO(
      get_logger(),
      "gimbal follow: %s -> %s, follow=%s, yaw clamp=%s [%.0f, %.0f] deg",
      base_frame_.c_str(), gimbal_frame_.c_str(),
      follow_enabled_ ? "on" : "off",
      allow_chassis_follow_ ? "off(chassis follows)" : "on",
      yaw_min_rad_ / kDegToRad, yaw_max_rad_ / kDegToRad);
  }

private:
  void OnPersons(const roborts_msgs::msg::PersonArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    persons_ = message->persons;
  }

  void OnLockTarget(const std_msgs::msg::UInt32::SharedPtr message)
  {
    locked_id_ = message->data;
    target_id_ = (message->data != 0) ? message->data : 0;
    if (locked_id_ == 0) {
      RCLCPP_INFO(get_logger(), "target lock released");
    } else {
      RCLCPP_INFO(get_logger(), "target lock: %u", locked_id_);
    }
  }

  void OnFollowEnable(const std_msgs::msg::Bool::SharedPtr message)
  {
    follow_enabled_ = message->data;
    RCLCPP_INFO(
      get_logger(), "visual following %s",
      follow_enabled_ ? "enabled" : "disabled");
  }

  void RequestControlMode()
  {
    if (mode_requested_ || !mode_client_) {
      if (mode_timer_) {
        mode_timer_->cancel();
      }
      return;
    }
    if (!mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for /set_gimbal_mode to switch the gimbal to CODE_CONTROL");
      return;
    }
    auto request = std::make_shared<roborts_msgs::srv::GimbalMode::Request>();
    request->gimbal_mode = static_cast<uint8_t>(control_mode_);
    mode_client_->async_send_request(request);
    mode_requested_ = true;
    mode_timer_->cancel();
    RCLCPP_INFO(
      get_logger(), "requested gimbal control mode %d", control_mode_);
  }

  bool LookupGimbal(double * yaw, double * pitch)
  {
    try {
      const geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform(
        base_frame_, gimbal_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      const auto & rotation = transform.transform.rotation;
      const tf2::Quaternion quaternion(
        rotation.x, rotation.y, rotation.z, rotation.w);
      double roll = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, *pitch, *yaw);
      return IsFinite(*yaw) && IsFinite(*pitch);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cannot look up %s -> %s: %s",
        base_frame_.c_str(), gimbal_frame_.c_str(), error.what());
      return false;
    }
  }

  /// Copy the selected person (locked id first, then current target, then the
  /// highest-score detection).  Caller holds no lock.
  bool SelectTarget(roborts_msgs::msg::Person * target)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (persons_.empty()) {
      return false;
    }
    const auto find_by_id = [this](uint32_t id) -> const roborts_msgs::msg::Person * {
        for (const auto & person : persons_) {
          if (person.id == id) {
            return &person;
          }
        }
        return nullptr;
      };

    const roborts_msgs::msg::Person * selected = nullptr;
    if (locked_id_ != 0) {
      // A lock means "follow this id"; do not silently switch to someone else
      // when it is temporarily out of view.
      selected = find_by_id(locked_id_);
      if (selected == nullptr) {
        return false;
      }
    } else if (target_id_ != 0) {
      selected = find_by_id(target_id_);
    }
    if (selected == nullptr) {
      for (const auto & person : persons_) {
        if (selected == nullptr || person.score > selected->score) {
          selected = &person;
        }
      }
    }
    if (selected == nullptr) {
      return false;
    }
    target_id_ = selected->id;
    *target = *selected;
    return true;
  }

  void ControlLoop()
  {
    if (!follow_enabled_) {
      return;
    }
    double current_yaw = 0.0;
    double current_pitch = 0.0;
    if (!LookupGimbal(&current_yaw, &current_pitch)) {
      return;
    }

    roborts_msgs::msg::Person target;
    const bool has_target = SelectTarget(&target);
    const double now = NowSeconds();
    if (has_target) {
      last_target_seen_ = now;
    }
    const double age = now - last_target_seen_;

    double desired_yaw = 0.0;
    double desired_pitch = 0.0;
    if (has_target) {
      if (std::abs(target.yaw) < deadband_rad_ &&
        std::abs(target.pitch) < deadband_rad_)
      {
        // Inside the deadband: keep the current absolute angle.
        return;
      }
      desired_yaw = current_yaw + yaw_gain_ * yaw_sign_ * target.yaw;
      desired_pitch = current_pitch + pitch_gain_ * pitch_sign_ * target.pitch;
    } else if (age > return_center_time_s_) {
      desired_yaw = center_yaw_rad_;
      desired_pitch = center_pitch_rad_;
    } else {
      // Hold the last angle through short detection dropouts.
      return;
    }

    const double dt = std::clamp(now - last_control_time_, 0.005, 0.2);
    last_control_time_ = now;
    const double max_step = max_rate_rad_s_ * dt;
    desired_yaw = current_yaw +
      std::clamp(desired_yaw - current_yaw, -max_step, max_step);
    desired_pitch = current_pitch +
      std::clamp(desired_pitch - current_pitch, -max_step, max_step);

    if (!allow_chassis_follow_) {
      const double clamped = std::clamp(desired_yaw, yaw_min_rad_, yaw_max_rad_);
      if (std::abs(clamped - desired_yaw) > 1e-4) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "target yaw %.1f deg is outside the gimbal range; not chasing the "
          "chassis", desired_yaw / kDegToRad);
      }
      desired_yaw = clamped;
    }
    desired_pitch = std::clamp(desired_pitch, pitch_min_rad_, pitch_max_rad_);

    const bool changed =
      std::isnan(last_cmd_yaw_) ||
      std::abs(desired_yaw - last_cmd_yaw_) > 0.003 ||
      std::abs(desired_pitch - last_cmd_pitch_) > 0.003;
    if (!changed && now - last_publish_ < 0.2) {
      return;
    }

    roborts_msgs::msg::GimbalAngle command;
    command.yaw_mode = false;      // absolute angle
    command.pitch_mode = false;    // absolute angle
    command.yaw_angle = desired_yaw;
    command.pitch_angle = desired_pitch;
    gimbal_pub_->publish(command);

    last_cmd_yaw_ = desired_yaw;
    last_cmd_pitch_ = desired_pitch;
    last_publish_ = now;
  }

  std::string base_frame_;
  std::string gimbal_frame_;
  bool follow_enabled_{false};

  double yaw_gain_{0.9};
  double pitch_gain_{0.9};
  double yaw_sign_{-1.0};
  double pitch_sign_{-1.0};
  double deadband_rad_{0.0};
  double max_rate_rad_s_{0.0};
  double hold_timeout_s_{0.5};
  double return_center_time_s_{3.0};
  double center_yaw_rad_{0.0};
  double center_pitch_rad_{0.0};
  bool allow_chassis_follow_{false};
  double yaw_min_rad_{0.0};
  double yaw_max_rad_{0.0};
  double pitch_min_rad_{0.0};
  double pitch_max_rad_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<roborts_msgs::msg::PersonArray>::SharedPtr persons_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr lock_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<roborts_msgs::msg::GimbalAngle>::SharedPtr gimbal_pub_;
  rclcpp::Client<roborts_msgs::srv::GimbalMode>::SharedPtr mode_client_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::mutex mutex_;
  std::vector<roborts_msgs::msg::Person> persons_;
  uint32_t locked_id_{0};
  uint32_t target_id_{0};
  int control_mode_{0};
  bool mode_requested_{false};

  double last_target_seen_{0.0};
  double last_control_time_{0.0};
  double last_publish_{-1.0};
  double last_cmd_yaw_{std::numeric_limits<double>::quiet_NaN()};
  double last_cmd_pitch_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace roborts_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<roborts_perception::GimbalFollowNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("gimbal_follow_node"), "fatal: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
