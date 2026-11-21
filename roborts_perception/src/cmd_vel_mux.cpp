/// P7: velocity arbiter.
///
/// Priority: safety > gesture > planner.  Each source expires after
/// source_timeout (0.5 s), after which the next priority is used; when nothing
/// is fresh the output is zero.  The output is published continuously at
/// publish_rate as roborts_msgs/TwistAccel on /cmd_vel_acc, which is what
/// roborts_base consumes.
///
/// When this node is active, remap the planner's output away from the base
/// topic, e.g. in the launch file:
///   remappings=[('/cmd_vel_acc', '/cmd_vel_acc_planner')]
/// Otherwise the planner and the mux both write /cmd_vel_acc.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <roborts_msgs/msg/twist_accel.hpp>

namespace roborts_perception
{

namespace
{

double NowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class CmdVelMux : public rclcpp::Node
{
public:
  CmdVelMux()
  : rclcpp::Node("cmd_vel_mux")
  {
    const std::string planner_topic =
      declare_parameter<std::string>("planner_topic", "/cmd_vel_acc_planner");
    const std::string gesture_topic =
      declare_parameter<std::string>("gesture_topic", "/cmd_vel_gesture");
    const std::string safety_topic =
      declare_parameter<std::string>("safety_topic", "/cmd_vel_safety");
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "/cmd_vel_acc");
    source_timeout_s_ = declare_parameter<double>("source_timeout", 0.5);
    const double publish_rate = declare_parameter<double>("publish_rate", 50.0);
    gesture_accel_linear_ =
      declare_parameter<double>("gesture_accel_linear", 1.0);
    gesture_accel_angular_ =
      declare_parameter<double>("gesture_accel_angular", 1.57);
    if (publish_rate <= 0.0 || source_timeout_s_ <= 0.0) {
      throw std::runtime_error(
              "publish_rate and source_timeout must be positive");
    }

    planner_sub_ = create_subscription<roborts_msgs::msg::TwistAccel>(
      planner_topic, rclcpp::QoS(1),
      std::bind(&CmdVelMux::OnPlanner, this, std::placeholders::_1));
    gesture_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      gesture_topic, rclcpp::QoS(1),
      std::bind(&CmdVelMux::OnGesture, this, std::placeholders::_1));
    safety_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      safety_topic, rclcpp::QoS(1),
      std::bind(&CmdVelMux::OnSafety, this, std::placeholders::_1));
    publisher_ = create_publisher<roborts_msgs::msg::TwistAccel>(
      output_topic, rclcpp::QoS(10));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = create_wall_timer(period, std::bind(&CmdVelMux::Tick, this));

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_mux: %s > %s > %s -> %s (timeout %.2f s)",
      safety_topic.c_str(), gesture_topic.c_str(), planner_topic.c_str(),
      output_topic.c_str(), source_timeout_s_);
  }

private:
  void OnPlanner(const roborts_msgs::msg::TwistAccel::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planner_ = *message;
    planner_time_ = NowSeconds();
    has_planner_ = true;
  }

  void OnGesture(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gesture_ = *message;
    gesture_time_ = NowSeconds();
    has_gesture_ = true;
  }

  void OnSafety(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    safety_ = *message;
    safety_time_ = NowSeconds();
    has_safety_ = true;
  }

  bool Fresh(bool has, double timestamp, double now) const
  {
    return has && (now - timestamp) <= source_timeout_s_;
  }

  void Tick()
  {
    roborts_msgs::msg::TwistAccel output;
    std::string source = "idle";

    std::lock_guard<std::mutex> lock(mutex_);
    const double now = NowSeconds();
    if (Fresh(has_safety_, safety_time_, now)) {
      FromTwist(safety_, false, &output);
      source = "safety";
    } else if (Fresh(has_gesture_, gesture_time_, now)) {
      FromTwist(gesture_, true, &output);
      source = "gesture";
    } else if (Fresh(has_planner_, planner_time_, now)) {
      output = planner_;
      source = "planner";
    }

    publisher_->publish(output);
    if (source != last_source_) {
      last_source_ = source;
      RCLCPP_INFO(get_logger(), "cmd_vel source: %s", source.c_str());
    }
  }

  void FromTwist(
    const geometry_msgs::msg::Twist & twist, bool allow_accel,
    roborts_msgs::msg::TwistAccel * output) const
  {
    output->twist = twist;
    const double linear = allow_accel ? gesture_accel_linear_ : 0.0;
    const double angular = allow_accel ? gesture_accel_angular_ : 0.0;
    output->accel.linear.x = linear;
    output->accel.linear.y = linear;
    output->accel.linear.z = 0.0;
    output->accel.angular.x = 0.0;
    output->accel.angular.y = 0.0;
    output->accel.angular.z = angular;
  }

  double source_timeout_s_{0.5};
  double gesture_accel_linear_{1.0};
  double gesture_accel_angular_{1.57};
  std::string last_source_{"idle"};

  std::mutex mutex_;
  roborts_msgs::msg::TwistAccel planner_;
  geometry_msgs::msg::Twist gesture_;
  geometry_msgs::msg::Twist safety_;
  double planner_time_{0.0};
  double gesture_time_{0.0};
  double safety_time_{0.0};
  bool has_planner_{false};
  bool has_gesture_{false};
  bool has_safety_{false};

  rclcpp::Subscription<roborts_msgs::msg::TwistAccel>::SharedPtr planner_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gesture_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safety_sub_;
  rclcpp::Publisher<roborts_msgs::msg::TwistAccel>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roborts_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<roborts_perception::CmdVelMux>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("cmd_vel_mux"), "fatal: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
