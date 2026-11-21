#include "roborts_perception/csi_camera.hpp"

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace roborts_perception
{
namespace
{

std::once_flag g_gst_init_once;

uint64_t NowNs()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

}  // namespace

CsiCamera::CsiCamera(int sensor_id, int width, int height, int framerate)
: sensor_id_(sensor_id), width_(width), height_(height), framerate_(framerate)
{
  std::call_once(g_gst_init_once, []() {gst_init(nullptr, nullptr);});
  fps_window_start_ = std::chrono::steady_clock::now();
}

CsiCamera::~CsiCamera()
{
  Stop();
}

std::string CsiCamera::PipelineDescription() const
{
  return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id_) +
         " ! video/x-raw(memory:NVMM),width=" + std::to_string(width_) +
         ",height=" + std::to_string(height_) +
         ",framerate=" + std::to_string(framerate_) + "/1" +
         " ! nvvidconv ! video/x-raw,format=BGRx" +
         " ! appsink name=sink max-buffers=1 drop=true sync=false"
         " emit-signals=true";
}

void CsiCamera::Start()
{
  if (pipeline_ != nullptr) {
    return;
  }

  stop_.store(false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = Frame();
    frame_ready_ = false;
    error_message_.clear();
  }
  frame_width_ = 0;
  frame_height_ = 0;
  sequence_.store(0);

  const std::string description = PipelineDescription();
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(description.c_str(), &error);
  if (pipeline_ == nullptr) {
    std::string message = "failed to build CSI pipeline";
    if (error != nullptr) {
      message += ": ";
      message += error->message;
      g_error_free(error);
    }
    message += "; pipeline was: " + description;
    throw std::runtime_error(message);
  }

  GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  if (sink == nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    throw std::runtime_error("CSI pipeline has no \"sink\" element");
  }
  g_signal_connect(sink, "new-sample", G_CALLBACK(&CsiCamera::OnNewSample), this);
  gst_object_unref(sink);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
    GST_STATE_CHANGE_FAILURE)
  {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    throw std::runtime_error(
            "failed to start CSI pipeline (check that the sensor is not in "
            "use by another process)");
  }

  bus_thread_ = std::thread(&CsiCamera::BusLoop, this);
}

void CsiCamera::Stop()
{
  if (stop_.exchange(true)) {
    return;
  }

  GstElement * pipeline = pipeline_;
  pipeline_ = nullptr;
  if (pipeline != nullptr) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
  }
  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = Frame();
    frame_ready_ = false;
  }
  frame_cv_.notify_all();

  if (pipeline != nullptr) {
    gst_object_unref(pipeline);
  }
}

std::string CsiCamera::ErrorMessage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return error_message_;
}

bool CsiCamera::ReadLatest(Frame * frame, int timeout_ms)
{
  std::unique_lock<std::mutex> lock(mutex_);
  frame_cv_.wait_for(
    lock, std::chrono::milliseconds(timeout_ms),
    [this]() {
      return frame_ready_ || !error_message_.empty() || stop_.load();
    });
  if (!frame_ready_) {
    return false;
  }
  *frame = std::move(latest_frame_);
  latest_frame_ = Frame();
  frame_ready_ = false;
  return true;
}

void CsiCamera::SetError(const std::string & text)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_message_.empty()) {
      error_message_ = text;
    }
  }
  frame_cv_.notify_all();
}

void CsiCamera::BusLoop()
{
  GstElement * pipeline = pipeline_;
  if (pipeline == nullptr) {
    return;
  }

  GstBus * bus = gst_element_get_bus(pipeline);
  while (!stop_.load()) {
    GstMessage * message = gst_bus_timed_pop_filtered(
      bus, 100 * GST_MSECOND,
      static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (message == nullptr) {
      continue;
    }

    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * error = nullptr;
      gchar * debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      std::string text = "GStreamer error: ";
      text += (error != nullptr) ? error->message : "unknown";
      if (debug != nullptr) {
        text += " (";
        text += debug;
        text += ")";
      }
      if (error != nullptr) {
        g_error_free(error);
      }
      g_free(debug);
      SetError(text);
    } else {
      SetError("CSI pipeline reached EOS");
    }
    gst_message_unref(message);
    break;
  }
  gst_object_unref(bus);
}

GstFlowReturn CsiCamera::OnNewSample(GstAppSink * sink, gpointer user_data)
{
  return static_cast<CsiCamera *>(user_data)->HandleSample(sink);
}

GstFlowReturn CsiCamera::HandleSample(GstAppSink * sink)
{
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (sample == nullptr) {
    return GST_FLOW_OK;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (buffer == nullptr) {
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  GstMapInfo map_info;
  if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  if (frame_width_ <= 0 || frame_height_ <= 0) {
    GstCaps * caps = gst_sample_get_caps(sample);
    GstStructure * structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &frame_width_);
    gst_structure_get_int(structure, "height", &frame_height_);
  }

  const int width = frame_width_;
  const int height = frame_height_;
  const size_t expected = static_cast<size_t>(width) * height * 4;
  if (width <= 0 || height <= 0 || map_info.size != expected) {
    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    SetError("unexpected BGRx buffer size in CSI frame");
    return GST_FLOW_OK;
  }

  Frame frame;
  const cv::Mat raw(height, width, CV_8UC4, map_info.data);
  cv::cvtColor(raw, frame.image, cv::COLOR_BGRA2BGR);
  gst_buffer_unmap(buffer, &map_info);
  gst_sample_unref(sample);

  frame.capture_time_ns = NowNs();
  frame.sequence = sequence_.fetch_add(1);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = std::move(frame);
    frame_ready_ = true;
  }
  frame_cv_.notify_one();
  UpdateFps();
  return GST_FLOW_OK;
}

void CsiCamera::UpdateFps()
{
  ++fps_window_count_;
  const auto now = std::chrono::steady_clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - fps_window_start_).count();
  if (elapsed >= 1.0) {
    fps_.store(fps_window_count_ / elapsed);
    fps_window_count_ = 0;
    fps_window_start_ = now;
  }
}

}  // namespace roborts_perception
