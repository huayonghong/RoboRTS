/// Gimbal CSI perception node.
///
/// * P3: reads IMX219 (CAM0) in-process through a GStreamer appsink -- no ROS
///   image topic is used between capture and processing;
/// * P4: runs yolo11n-pose on TensorRT (FP16 engine) and publishes
///   /perception/persons;
/// * P5: optionally runs a hand-pose engine on wrist ROIs, classifies gestures
///   with rules and publishes /perception/gestures (enable_gesture).
///
/// Capture and processing run on separate threads, so a slow consumer drops
/// frames instead of building latency.  No PyTorch/ultralytics import exists
/// in this process; inference is TensorRT only.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <roborts_msgs/msg/gesture_array.hpp>
#include <roborts_msgs/msg/person_array.hpp>

#include "roborts_perception/camera_model.hpp"
#include "roborts_perception/csi_camera.hpp"
#include "roborts_perception/gesture_classifier.hpp"
#include "roborts_perception/person_tracker.hpp"
#include "roborts_perception/pose_detector.hpp"

namespace roborts_perception
{

namespace
{

builtin_interfaces::msg::Time StampFromNs(uint64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
  return stamp;
}

double NowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string DefaultModelPath(const std::string & filename)
{
  try {
    return ament_index_cpp::get_package_share_directory("roborts_perception") +
           "/models/" + filename;
  } catch (const std::exception &) {
    return "models/" + filename;
  }
}

struct GestureCandidate
{
  uint8_t id{kGestureNone};
  float score{0.0f};
  cv::Rect2f box;
};

/// Every hand detection of the current frame, for the debug overlay (the
/// confirmed gesture is carried by GestureArray separately).
struct HandDebug
{
  cv::Rect2f box;
  std::vector<cv::Point2f> keypoints;
  std::vector<float> keypoint_scores;
};

struct GestureState
{
  uint8_t candidate_id{kGestureNone};
  int count{0};
  uint8_t held_id{kGestureNone};
  bool has_emitted{false};
  double last_emit{-1.0};
};

}  // namespace

class PersonGestureNode : public rclcpp::Node
{
public:
  PersonGestureNode()
  : rclcpp::Node("person_gesture_node")
  {
    sensor_id_ = declare_parameter<int>("sensor_id", 0);
    image_width_ = declare_parameter<int>("image_width", 1920);
    image_height_ = declare_parameter<int>("image_height", 1080);
    framerate_ = declare_parameter<int>("framerate", 30);
    frame_id_ = declare_parameter<std::string>("frame_id", "csi_camera_optical_frame");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    debug_image_fps_ = declare_parameter<double>("debug_image_fps", 10.0);
    stats_period_ = declare_parameter<double>("stats_period", 10.0);
    const std::string camera_info_path =
      declare_parameter<std::string>("camera_info_path", "");

    enable_person_ = declare_parameter<bool>("enable_person", true);
    enable_gesture_ = declare_parameter<bool>("enable_gesture", false);
    person_engine_path_ = declare_parameter<std::string>("person_engine_path", "");
    hand_engine_path_ = declare_parameter<std::string>("hand_engine_path", "");
    person_conf_threshold_ = declare_parameter<double>("person_conf_threshold", 0.25);
    person_nms_iou_ = declare_parameter<double>("person_nms_iou", 0.45);
    min_person_height_px_ = declare_parameter<double>("min_person_height_px", 40.0);
    keypoint_conf_threshold_ =
      declare_parameter<double>("keypoint_conf_threshold", 0.3);
    min_valid_keypoints_ = declare_parameter<int>("min_valid_keypoints", 3);
    person_height_m_ = declare_parameter<double>("person_height_m", 1.7);
    hand_conf_threshold_ = declare_parameter<double>("hand_conf_threshold", 0.3);
    hand_roi_scale_ = declare_parameter<double>("hand_roi_scale", 0.45);
    gesture_min_frames_ = declare_parameter<int>("gesture_min_frames", 5);
    gesture_min_score_ = declare_parameter<double>("gesture_min_score", 0.6);
    gesture_cooldown_s_ = declare_parameter<double>("gesture_cooldown_s", 1.5);

    if (stats_period_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(), "stats_period=%.1f is invalid, using 10 s", stats_period_);
      stats_period_ = 10.0;
    }
    if (publish_debug_image_) {
      debug_period_ = (debug_image_fps_ > 0.0) ? 1.0 / debug_image_fps_ : 0.0;
    }

    LoadCameraIntrinsics(camera_info_path);
    CreateDebugPublishers();
    CreatePerceptionPublishers();
    InitDetectors();

    camera_ = std::make_unique<CsiCamera>(
      sensor_id_, image_width_, image_height_, framerate_);
    try {
      camera_->Start();
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "cannot start CSI camera: %s", error.what());
      throw;
    }

    Frame first_frame;
    if (!camera_->ReadLatest(&first_frame, 3000)) {
      const std::string detail = camera_->ErrorMessage();
      camera_->Stop();
      throw std::runtime_error(
              "no CSI frame received within 3 s; check /dev/video0, the Argus "
              "plugins and that no other process is using the camera" +
              (detail.empty() ? std::string() : " (" + detail + ")"));
    }

    processing_thread_ = std::thread(&PersonGestureNode::ProcessingLoop, this);

    const auto stats_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(stats_period_));
    stats_timer_ = create_wall_timer(stats_period, [this]() {LogStats();});

    RCLCPP_INFO(
      get_logger(),
      "CSI camera started: sensor-id=%d, %dx%d@%d, frame_id=\"%s\", "
      "debug_image=%s, person=%s, gesture=%s", sensor_id_, image_width_,
      image_height_, framerate_, frame_id_.c_str(),
      publish_debug_image_ ? "true" : "false", enable_person_ ? "true" : "false",
      enable_gesture_ ? "true" : "false");
    if (!publish_debug_image_) {
      RCLCPP_INFO(
        get_logger(),
        "no image topics published (set publish_debug_image:=true for "
        "/perception/debug/image_raw)");
    }
  }

  ~PersonGestureNode() override
  {
    Shutdown();
  }

  void Shutdown()
  {
    if (shutdown_requested_.exchange(true)) {
      return;
    }
    if (camera_) {
      camera_->Stop();
    }
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
  }

private:
  // ---------------------------------------------------------------- set-up
  void LoadCameraIntrinsics(const std::string & configured_path)
  {
    std::string path = configured_path;
    if (path.empty()) {
      try {
        path = ament_index_cpp::get_package_share_directory("roborts_perception") +
          "/config/camera_csi.yaml";
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(), "cannot resolve package share directory: %s",
          error.what());
        return;
      }
    }

    try {
      camera_model_ = LoadCameraModel(path);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(),
        "%s; capture runs without projection, run "
        "\"ros2 run roborts_perception calibrate_csi\" before using "
        "pixel->bearing", error.what());
      camera_model_.reset();
      return;
    }

    if (!camera_model_->calibrated) {
      RCLCPP_WARN(
        get_logger(),
        "%s contains NOMINAL (uncalibrated) intrinsics; run "
        "\"ros2 run roborts_perception calibrate_csi\" and copy the result "
        "over this file before using pixel->bearing", path.c_str());
    }
    if (camera_model_->image_width != image_width_ ||
      camera_model_->image_height != image_height_)
    {
      RCLCPP_WARN(
        get_logger(),
        "camera intrinsics are for %dx%d, but capture is %dx%d; recalibrate "
        "at the capture resolution", camera_model_->image_width,
        camera_model_->image_height, image_width_, image_height_);
    } else {
      RCLCPP_INFO(
        get_logger(), "camera intrinsics loaded from %s (fx=%.1f, fy=%.1f, "
        "calibrated=%s)", path.c_str(), camera_model_->fx(), camera_model_->fy(),
        camera_model_->calibrated ? "true" : "false");
    }
  }

  void CreateDebugPublishers()
  {
    if (!publish_debug_image_) {
      return;
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/perception/debug/image_raw", qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/perception/debug/camera_info", qos);
  }

  void CreatePerceptionPublishers()
  {
    if (enable_person_) {
      persons_pub_ = create_publisher<roborts_msgs::msg::PersonArray>(
        "/perception/persons", rclcpp::QoS(10));
    }
    if (enable_gesture_) {
      gestures_pub_ = create_publisher<roborts_msgs::msg::GestureArray>(
        "/perception/gestures", rclcpp::QoS(10));
    }
  }

  void InitDetectors()
  {
    if (enable_person_) {
      const std::string path = person_engine_path_.empty() ?
        DefaultModelPath("yolo11n-pose_fp16.engine") : person_engine_path_;
      PoseDecodeOptions options;
      options.conf_threshold = static_cast<float>(person_conf_threshold_);
      options.iou_threshold = static_cast<float>(person_nms_iou_);
      options.max_detections = 30;
      options.expected_keypoints = 17;
      options.min_box_height_px = static_cast<float>(min_person_height_px_);
      options.keypoint_conf_threshold =
        static_cast<float>(keypoint_conf_threshold_);
      options.min_valid_keypoints = min_valid_keypoints_;
      try {
        pose_detector_ = std::make_unique<PoseDetector>(path, options);
        tracker_ = std::make_unique<PersonTracker>();
        RCLCPP_INFO(
          get_logger(), "person detector: %s (%dx%d, %d keypoints)", path.c_str(),
          pose_detector_->input_size(), pose_detector_->input_size(),
          pose_detector_->num_keypoints());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(),
          "person detection disabled: %s. Build the engine with "
          "\"ros2 run roborts_perception build_models --pose\"",
          error.what());
        enable_person_ = false;
      }
    }

    if (enable_gesture_) {
      if (!enable_person_) {
        RCLCPP_ERROR(
          get_logger(), "gesture detection needs person detection (rolls "
          "wrist keypoints); disabling gesture branch");
        enable_gesture_ = false;
        return;
      }
      const std::string path = hand_engine_path_.empty() ?
        DefaultModelPath("hand_pose_fp16.engine") : hand_engine_path_;
      PoseDecodeOptions options;
      options.conf_threshold = static_cast<float>(hand_conf_threshold_);
      options.iou_threshold = 0.45f;
      options.max_detections = 5;
      options.expected_keypoints = 21;
      options.min_box_height_px = 0.0f;
      options.keypoint_conf_threshold = 0.3f;
      options.min_valid_keypoints = 5;
      try {
        hand_detector_ = std::make_unique<PoseDetector>(path, options);
        RCLCPP_INFO(
          get_logger(), "hand detector: %s (%dx%d, %d keypoints)", path.c_str(),
          hand_detector_->input_size(), hand_detector_->input_size(),
          hand_detector_->num_keypoints());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(),
          "gesture detection disabled: %s. Build the engine with "
          "\"ros2 run roborts_perception build_models --hand-weights "
          "/path/to/hand_best.pt\"", error.what());
        enable_gesture_ = false;
        gestures_pub_.reset();
      }
    }
  }

  // -------------------------------------------------------------- inference
  void ProcessingLoop()
  {
    while (rclcpp::ok() && !shutdown_requested_.load()) {
      Frame frame;
      if (!camera_->ReadLatest(&frame, 500)) {
        ReportCameraError();
        continue;
      }
      processed_frames_.fetch_add(1);
      processed_since_stats_.fetch_add(1);
      try {
        ProcessFrame(frame);
      } catch (const std::exception & error) {
        if (!rclcpp::ok() || shutdown_requested_.load()) {
          break;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_error_log_ >= std::chrono::seconds(5)) {
          last_error_log_ = now;
          RCLCPP_ERROR(get_logger(), "frame processing failed: %s", error.what());
        }
      }
    }
  }

  void ProcessFrame(const Frame & frame)
  {
    const builtin_interfaces::msg::Time stamp = StampFromNs(frame.capture_time_ns);

    std::vector<TrackedPerson> tracked;
    if (enable_person_) {
      const std::vector<PoseDetection> detections =
        pose_detector_->Infer(frame.image);
      tracked = tracker_->Update(detections);
      last_person_count_.store(static_cast<int>(tracked.size()));
      last_person_infer_ms_.store(pose_detector_->last_inference_ms());

      roborts_msgs::msg::PersonArray persons;
      persons.header.stamp = stamp;
      persons.header.frame_id = frame_id_;
      persons.persons.reserve(tracked.size());
      for (const TrackedPerson & person : tracked) {
        persons.persons.push_back(ToPersonMessage(person, stamp));
      }
      persons_pub_->publish(persons);
    }

    roborts_msgs::msg::GestureArray gestures;
    gestures.header.stamp = stamp;
    gestures.header.frame_id = frame_id_;
    if (enable_gesture_) {
      gestures = BuildGestures(tracked, frame.image, stamp);
      gestures_pub_->publish(gestures);
    }

    if (publish_debug_image_) {
      MaybePublishDebug(frame, tracked, gestures);
    }
  }

  roborts_msgs::msg::Person ToPersonMessage(
    const TrackedPerson & person,
    const builtin_interfaces::msg::Time & stamp) const
  {
    roborts_msgs::msg::Person message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.id = person.id;
    message.score = person.score;
    message.center.x = person.box.x + person.box.width * 0.5;
    message.center.y = person.box.y + person.box.height * 0.5;
    message.center.z = 0.0;
    message.size.x = person.box.width;
    message.size.y = person.box.height;
    message.size.z = 0.0;

    if (camera_model_) {
      const cv::Point2d bearing =
        camera_model_->PixelToBearing(message.center.x, message.center.y);
      message.yaw = static_cast<float>(bearing.x);
      message.pitch = static_cast<float>(bearing.y);
      if (person.box.height > 1.0f) {
        const double distance =
          camera_model_->fy() * person_height_m_ / person.box.height;
        message.distance = static_cast<float>(
          std::max(0.2, std::min(distance, 20.0)));
      }
    }
    return message;
  }

  // --------------------------------------------------------------- gestures
  GestureCandidate DetectHandGesture(
    const TrackedPerson & person, const cv::Mat & image)
  {
    GestureCandidate best;
    if (!hand_detector_) {
      return best;
    }
    // COCO wrist/elbow indices: 9/7 = left, 10/8 = right.
    const int wrists[2] = {9, 10};
    const int elbows[2] = {7, 8};
    for (int side = 0; side < 2; ++side) {
      const int wrist_index = wrists[side];
      const int elbow_index = elbows[side];
      if (wrist_index >= static_cast<int>(person.keypoints.size()) ||
        person.keypoint_scores[wrist_index] < 0.3f)
      {
        continue;
      }
      const cv::Point2f & wrist = person.keypoints[wrist_index];
      cv::Point2f center = wrist;
      float size = static_cast<float>(hand_roi_scale_ * person.box.height);
      // The hand extends beyond the wrist along the forearm direction, so
      // shift the ROI outward when the elbow is visible.  This matters a lot
      // for close-up hands where the palm is much larger than the person-pose
      // wrist neighbourhood.
      if (elbow_index < static_cast<int>(person.keypoints.size()) &&
        person.keypoint_scores[elbow_index] >= 0.3f)
      {
        const cv::Point2f & elbow = person.keypoints[elbow_index];
        const float forearm = cv::norm(wrist - elbow);
        if (forearm > 1.0f) {
          center = wrist + (wrist - elbow) * 0.6f;
          size = std::max(2.5f * forearm, size);
        }
      }
      size = std::max(64.0f, std::min(size, 800.0f));
      cv::Rect roi(
        static_cast<int>(center.x - size * 0.5f),
        static_cast<int>(center.y - size * 0.5f),
        static_cast<int>(size), static_cast<int>(size));
      roi &= cv::Rect(0, 0, image.cols, image.rows);
      if (roi.width < 32 || roi.height < 32) {
        continue;
      }

      const std::vector<PoseDetection> hands =
        hand_detector_->Infer(image(roi));
      for (const PoseDetection & hand : hands) {
        std::vector<cv::Point2f> keypoints = hand.keypoints;
        for (cv::Point2f & point : keypoints) {
          point.x += static_cast<float>(roi.x);
          point.y += static_cast<float>(roi.y);
        }
        HandDebug debug;
        debug.box = cv::Rect2f(
          hand.box.x + roi.x, hand.box.y + roi.y,
          hand.box.width, hand.box.height);
        debug.keypoints = keypoints;
        debug.keypoint_scores = hand.keypoint_scores;
        last_hand_debug_.push_back(std::move(debug));

        const GestureResult result =
          ClassifyGesture(keypoints, hand.keypoint_scores);
        if (result.id == kGestureNone || result.score < gesture_min_score_) {
          continue;
        }
        if (result.score > best.score) {
          best.id = result.id;
          best.score = result.score;
          best.box = cv::Rect2f(
            hand.box.x + roi.x, hand.box.y + roi.y,
            hand.box.width, hand.box.height);
        }
      }
    }
    return best;
  }

  roborts_msgs::msg::GestureArray BuildGestures(
    const std::vector<TrackedPerson> & tracked, const cv::Mat & image,
    const builtin_interfaces::msg::Time & stamp)
  {
    roborts_msgs::msg::GestureArray message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    last_hand_debug_.clear();

    const double start = NowSeconds();
    std::unordered_map<uint32_t, GestureCandidate> candidates;
    std::unordered_set<uint32_t> current_ids;
    for (const TrackedPerson & person : tracked) {
      current_ids.insert(person.id);
      GestureCandidate candidate = DetectHandGesture(person, image);
      if (candidate.id != kGestureNone) {
        candidates[person.id] = candidate;
      }
    }
    last_hand_infer_ms_.store((NowSeconds() - start) * 1000.0);

    for (auto it = gesture_states_.begin(); it != gesture_states_.end();) {
      if (current_ids.count(it->first) == 0) {
        it = gesture_states_.erase(it);
      } else {
        ++it;
      }
    }

    const double now = NowSeconds();
    for (const TrackedPerson & person : tracked) {
      GestureState & state = gesture_states_[person.id];
      const auto candidate_it = candidates.find(person.id);
      const uint8_t candidate_id = (candidate_it != candidates.end()) ?
        candidate_it->second.id : static_cast<uint8_t>(kGestureNone);

      if (candidate_id == state.candidate_id) {
        ++state.count;
      } else {
        state.candidate_id = candidate_id;
        state.count = 1;
      }

      if (candidate_id == kGestureNone) {
        state.held_id = kGestureNone;
      } else if (state.count >= gesture_min_frames_ &&
        candidate_id != state.held_id &&
        (!state.has_emitted || now - state.last_emit >= gesture_cooldown_s_))
      {
        state.held_id = candidate_id;
        state.has_emitted = true;
        state.last_emit = now;
        RCLCPP_INFO(
          get_logger(), "person %u gesture \"%s\" (%.2f)", person.id,
          GestureName(state.held_id), candidate_it->second.score);
      }

      if (state.held_id == kGestureNone || candidate_it == candidates.end() ||
        candidate_id != state.held_id)
      {
        continue;
      }

      roborts_msgs::msg::Gesture gesture;
      gesture.header = message.header;
      gesture.track_id = person.id;
      gesture.gesture_id = state.held_id;
      gesture.name = GestureName(state.held_id);
      gesture.score = candidate_it->second.score;
      gesture.center.x = candidate_it->second.box.x +
        candidate_it->second.box.width * 0.5;
      gesture.center.y = candidate_it->second.box.y +
        candidate_it->second.box.height * 0.5;
      gesture.center.z = 0.0;
      gesture.size.x = candidate_it->second.box.width;
      gesture.size.y = candidate_it->second.box.height;
      gesture.size.z = 0.0;
      message.gestures.push_back(gesture);
    }
    return message;
  }

  // ------------------------------------------------------------------ debug
  void MaybePublishDebug(
    const Frame & frame, const std::vector<TrackedPerson> & tracked,
    const roborts_msgs::msg::GestureArray & gestures)
  {
    const auto now = std::chrono::steady_clock::now();
    if (debug_period_ > 0.0 && now < next_debug_time_) {
      return;
    }
    next_debug_time_ = (debug_period_ > 0.0) ?
      now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(debug_period_)) : now;

    cv::Mat debug = frame.image.clone();
    for (const TrackedPerson & person : tracked) {
      const cv::Rect box(
        static_cast<int>(person.box.x), static_cast<int>(person.box.y),
        static_cast<int>(person.box.width), static_cast<int>(person.box.height));
      cv::rectangle(debug, box, cv::Scalar(0, 255, 0), 2);
      cv::putText(
        debug, cv::format("id %u %.2f", person.id, person.score),
        cv::Point(box.x, std::max(15, box.y - 6)), cv::FONT_HERSHEY_SIMPLEX,
        0.6, cv::Scalar(0, 255, 0), 2);
      for (size_t k = 0; k < person.keypoints.size(); ++k) {
        if (person.keypoint_scores[k] < static_cast<float>(
            keypoint_conf_threshold_))
        {
          continue;
        }
        cv::circle(
          debug, person.keypoints[k], 3, cv::Scalar(0, 255, 255), -1);
      }
    }
    for (const auto & gesture : gestures.gestures) {
      cv::putText(
        debug, gesture.name,
        cv::Point(static_cast<int>(gesture.center.x),
        static_cast<int>(gesture.center.y)), cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(255, 255, 0), 2);
    }
    for (const HandDebug & hand : last_hand_debug_) {
      cv::rectangle(debug, cv::Rect(hand.box), cv::Scalar(255, 255, 0), 1);
      for (size_t k = 0; k < hand.keypoints.size(); ++k) {
        if (hand.keypoint_scores.size() > k &&
          hand.keypoint_scores[k] < 0.3f)
        {
          continue;
        }
        cv::circle(debug, hand.keypoints[k], 3, cv::Scalar(255, 255, 0), -1);
      }
    }

    const builtin_interfaces::msg::Time stamp = StampFromNs(frame.capture_time_ns);
    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = stamp;
    image_msg.header.frame_id = frame_id_;
    image_msg.height = static_cast<uint32_t>(debug.rows);
    image_msg.width = static_cast<uint32_t>(debug.cols);
    image_msg.encoding = "bgr8";
    image_msg.is_bigendian = false;
    image_msg.step = static_cast<uint32_t>(debug.cols * debug.channels());
    const size_t bytes = debug.total() * debug.elemSize();
    image_msg.data.resize(bytes);
    std::memcpy(image_msg.data.data(), debug.data, bytes);
    image_pub_->publish(image_msg);

    if (camera_model_ && camera_info_pub_) {
      camera_info_pub_->publish(BuildCameraInfo(stamp));
    }
  }

  sensor_msgs::msg::CameraInfo BuildCameraInfo(
    const builtin_interfaces::msg::Time & stamp) const
  {
    const CameraModel & model = *camera_model_;
    sensor_msgs::msg::CameraInfo message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.height = static_cast<uint32_t>(model.image_height);
    message.width = static_cast<uint32_t>(model.image_width);
    message.distortion_model = model.distortion_model;
    message.d = model.distortion_coefficients;

    const std::array<double, 9> k{
      model.fx(), 0.0, model.cx(),
      0.0, model.fy(), model.cy(),
      0.0, 0.0, 1.0};
    const std::array<double, 9> r{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
    const std::array<double, 12> p{
      model.fx(), 0.0, model.cx(), 0.0,
      0.0, model.fy(), model.cy(), 0.0,
      0.0, 0.0, 1.0, 0.0};
    message.k = k;
    message.r = r;
    message.p = p;
    return message;
  }

  // ------------------------------------------------------------------ stats
  void ReportCameraError()
  {
    const std::string error = camera_->ErrorMessage();
    if (error.empty()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_error_log_ < std::chrono::seconds(5)) {
      return;
    }
    last_error_log_ = now;
    RCLCPP_ERROR(
      get_logger(), "CSI capture problem: %s; restart the node after fixing it",
      error.c_str());
  }

  void LogStats()
  {
    const uint64_t sequence = camera_->FrameCount();
    const uint64_t captured = sequence - last_stats_sequence_;
    last_stats_sequence_ = sequence;
    const uint64_t processed = processed_since_stats_.exchange(0);

    if (captured == 0) {
      RCLCPP_WARN(
        get_logger(), "no CSI frames in the last %.1f s (driver fps %.1f)",
        stats_period_, camera_->MeasuredFps());
      return;
    }

    std::string extra;
    if (enable_person_) {
      extra = cv::format(
        ", persons %d, pose %.1f ms", last_person_count_.load(),
        last_person_infer_ms_.load());
      if (enable_gesture_) {
        extra += cv::format(", hand %.1f ms", last_hand_infer_ms_.load());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "CSI %dx%d: driver %.1f FPS, captured %" PRIu64 " in %.1f s, "
      "processed %" PRIu64 " (%.1f FPS), skipped %" PRIu64 "%s",
      image_width_, image_height_, camera_->MeasuredFps(), captured,
      stats_period_, processed, processed / stats_period_,
      captured - processed, extra.c_str());
  }

  int sensor_id_{0};
  int image_width_{1920};
  int image_height_{1080};
  int framerate_{30};
  std::string frame_id_;
  bool publish_debug_image_{false};
  double debug_image_fps_{10.0};
  double stats_period_{10.0};
  double debug_period_{0.0};

  bool enable_person_{true};
  bool enable_gesture_{false};
  std::string person_engine_path_;
  std::string hand_engine_path_;
  double person_conf_threshold_{0.25};
  double person_nms_iou_{0.45};
  double min_person_height_px_{40.0};
  double keypoint_conf_threshold_{0.3};
  int min_valid_keypoints_{3};
  double person_height_m_{1.7};
  double hand_conf_threshold_{0.3};
  double hand_roi_scale_{0.45};
  int gesture_min_frames_{5};
  double gesture_min_score_{0.6};
  double gesture_cooldown_s_{1.5};

  std::optional<CameraModel> camera_model_;
  std::unique_ptr<CsiCamera> camera_;
  std::unique_ptr<PoseDetector> pose_detector_;
  std::unique_ptr<PersonTracker> tracker_;
  std::unique_ptr<PoseDetector> hand_detector_;
  std::unordered_map<uint32_t, GestureState> gesture_states_;
  std::vector<HandDebug> last_hand_debug_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<roborts_msgs::msg::PersonArray>::SharedPtr persons_pub_;
  rclcpp::Publisher<roborts_msgs::msg::GestureArray>::SharedPtr gestures_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::thread processing_thread_;
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<uint64_t> processed_frames_{0};
  std::atomic<uint64_t> processed_since_stats_{0};
  std::atomic<int> last_person_count_{0};
  std::atomic<double> last_person_infer_ms_{0.0};
  std::atomic<double> last_hand_infer_ms_{0.0};
  uint64_t last_stats_sequence_{0};
  std::chrono::steady_clock::time_point next_debug_time_{};
  std::chrono::steady_clock::time_point last_error_log_{};
};

}  // namespace roborts_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<roborts_perception::PersonGestureNode>();
    rclcpp::spin(node);
    node->Shutdown();
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("person_gesture_node"), "fatal: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
