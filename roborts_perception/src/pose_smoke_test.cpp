/// Offline smoke test for a pose TensorRT engine.
///
/// Useful on the robot to verify an engine and the decoder without the camera:
///   ros2 run roborts_perception pose_smoke_test IMAGE ENGINE [keypoints] [iterations]
/// Example:
///   ros2 run roborts_perception pose_smoke_test /tmp/bus.jpg ENGINE 17
///
/// Writes an annotated copy to /tmp/pose_smoke_test.jpg.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "roborts_perception/pose_detector.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(
      stderr, "usage: %s IMAGE ENGINE [keypoints=17]\n", argv[0]);
    return 2;
  }
  rclcpp::init(argc, argv);

  roborts_perception::PoseDecodeOptions options;
  options.expected_keypoints = (argc > 3) ? std::atoi(argv[3]) : 17;
  options.min_box_height_px = 0.0f;
  options.min_valid_keypoints = 3;

  try {
    roborts_perception::PoseDetector detector(argv[2], options);
    const cv::Mat image = cv::imread(argv[1]);
    if (image.empty()) {
      std::fprintf(stderr, "error: cannot read %s\n", argv[1]);
      rclcpp::shutdown();
      return 1;
    }
    const std::vector<roborts_perception::PoseDetection> detections =
      [&]() {
        const int iterations = (argc > 4) ? std::atoi(argv[4]) : 5;
        std::vector<roborts_perception::PoseDetection> result;
        double best_ms = 1e9;
        double total_ms = 0.0;
        for (int i = 0; i < iterations; ++i) {
          result = detector.Infer(image);
          const double ms = detector.last_inference_ms();
          best_ms = std::min(best_ms, ms);
          total_ms += ms;
        }
        std::printf(
          "inference: %.2f ms last, %.2f ms best, %.2f ms average "
          "(%d iterations)\n", detector.last_inference_ms(), best_ms,
          total_ms / iterations, iterations);
        return result;
      }();
    std::printf(
      "engine %s: %dx%d input, %d keypoints, %zu detections\n",
      argv[2], detector.input_size(), detector.input_size(),
      detector.num_keypoints(), detections.size());

    cv::Mat annotated = image.clone();
    for (const auto & detection : detections) {
      std::printf(
        "  score %.3f box [%.1f, %.1f, %.1f, %.1f]\n", detection.score,
        detection.box.x, detection.box.y, detection.box.width,
        detection.box.height);
      cv::rectangle(annotated, detection.box, cv::Scalar(0, 255, 0), 2);
      for (size_t k = 0; k < detection.keypoints.size(); ++k) {
        if (detection.keypoint_scores[k] < 0.3f) {
          continue;
        }
        cv::circle(
          annotated, detection.keypoints[k], 4, cv::Scalar(0, 255, 255), -1);
      }
    }
    cv::imwrite("/tmp/pose_smoke_test.jpg", annotated);
    std::printf("wrote /tmp/pose_smoke_test.jpg\n");
  } catch (const std::exception & error) {
    std::fprintf(stderr, "error: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
