#ifndef ROBORTS_PERCEPTION__CSI_CAMERA_HPP_
#define ROBORTS_PERCEPTION__CSI_CAMERA_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <opencv2/core.hpp>

namespace roborts_perception
{

struct Frame
{
  cv::Mat image;              // BGR, CV_8UC3
  uint64_t capture_time_ns{0};
  uint64_t sequence{0};
};

/// Argus CSI camera with a single-slot "latest frame" buffer.
///
/// The GStreamer pipeline is:
///   nvarguscamerasrc ! video/x-raw(memory:NVMM),... ! nvvidconv !
///   video/x-raw,format=BGRx ! appsink max-buffers=1 drop=true
///
/// The appsink callback only converts BGRx -> BGR and replaces the latest
/// frame.  Consumers call ReadLatest() at whatever rate they can sustain;
/// because the sink keeps one buffer and drops the oldest on overflow, a slow
/// consumer skips frames instead of accumulating latency.
class CsiCamera
{
public:
  CsiCamera(int sensor_id, int width, int height, int framerate);
  ~CsiCamera();

  CsiCamera(const CsiCamera &) = delete;
  CsiCamera & operator=(const CsiCamera &) = delete;

  /// Build and start the pipeline.  Throws std::runtime_error on failure.
  void Start();

  /// Stop the pipeline and join the bus thread.  Idempotent.
  void Stop();

  /// Move the newest frame captured since the previous call into frame.
  /// Blocks up to timeout_ms.  Returns false on timeout or pipeline error.
  bool ReadLatest(Frame * frame, int timeout_ms);

  double MeasuredFps() const {return fps_.load();}
  uint64_t FrameCount() const {return sequence_.load();}
  std::string ErrorMessage() const;

private:
  static GstFlowReturn OnNewSample(GstAppSink * sink, gpointer user_data);
  GstFlowReturn HandleSample(GstAppSink * sink);
  void BusLoop();
  void UpdateFps();
  void SetError(const std::string & text);
  std::string PipelineDescription() const;

  const int sensor_id_;
  const int width_;
  const int height_;
  const int framerate_;

  GstElement * pipeline_{nullptr};
  std::thread bus_thread_;
  std::atomic<bool> stop_{false};

  mutable std::mutex mutex_;
  std::condition_variable frame_cv_;
  Frame latest_frame_;
  bool frame_ready_{false};
  std::string error_message_;
  int frame_width_{0};
  int frame_height_{0};

  std::atomic<uint64_t> sequence_{0};
  std::atomic<double> fps_{0.0};
  std::chrono::steady_clock::time_point fps_window_start_;
  int fps_window_count_{0};
};

}  // namespace roborts_perception

#endif  // ROBORTS_PERCEPTION__CSI_CAMERA_HPP_
