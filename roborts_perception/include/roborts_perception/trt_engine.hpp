#ifndef ROBORTS_PERCEPTION__TRT_ENGINE_HPP_
#define ROBORTS_PERCEPTION__TRT_ENGINE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>
#include <NvInfer.h>

namespace roborts_perception
{

struct TensorInfo
{
  std::string name;
  nvinfer1::Dims dims{};
  nvinfer1::DataType dtype{nvinfer1::DataType::kFLOAT};
  bool is_input{false};
  size_t element_count{0};
  size_t byte_size{0};
};

/// Minimal synchronous TensorRT engine wrapper (TRT 10 API).
///
/// The engine must have exactly one input and at least one output tensor, all
/// with float32 I/O.  Engines built with trtexec's default I/O formats satisfy
/// this even when `--fp16` is used (only the network internals are FP16).
class TrtEngine
{
public:
  explicit TrtEngine(const std::string & engine_path, int device_id = 0);
  ~TrtEngine();

  TrtEngine(const TrtEngine &) = delete;
  TrtEngine & operator=(const TrtEngine &) = delete;

  /// Run one inference.  input/output sizes must match the tensor sizes.
  void Infer(const std::vector<float> & input, std::vector<float> * output);

  const TensorInfo & input_info() const {return input_info_;}
  const TensorInfo & output_info() const {return output_info_;}
  double last_inference_ms() const {return last_inference_ms_;}

private:
  class Logger;

  void LoadEngine(const std::string & engine_path);
  void Cleanup();
  TensorInfo DescribeTensor(const char * name) const;

  std::unique_ptr<Logger> logger_;
  nvinfer1::IRuntime * runtime_{nullptr};
  nvinfer1::ICudaEngine * engine_{nullptr};
  nvinfer1::IExecutionContext * context_{nullptr};
  cudaStream_t stream_{nullptr};
  void * device_input_{nullptr};
  void * device_output_{nullptr};
  TensorInfo input_info_;
  TensorInfo output_info_;
  double last_inference_ms_{0.0};
};

}  // namespace roborts_perception

#endif  // ROBORTS_PERCEPTION__TRT_ENGINE_HPP_
