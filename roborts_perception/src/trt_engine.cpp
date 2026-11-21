#include "roborts_perception/trt_engine.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace roborts_perception
{

namespace
{

size_t ElementCount(const nvinfer1::Dims & dims, const std::string & name)
{
  if (dims.nbDims <= 0) {
    throw std::runtime_error("tensor \"" + name + "\" has no dimensions");
  }
  size_t count = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      throw std::runtime_error(
              "tensor \"" + name + "\" has a non-static dimension");
    }
    count *= static_cast<size_t>(dims.d[i]);
  }
  return count;
}

void CheckCuda(cudaError_t status, const char * what)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(
            std::string(what) + " failed: " + cudaGetErrorString(status));
  }
}

}  // namespace

class TrtEngine::Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      RCLCPP_WARN(rclcpp::get_logger("tensorrt"), "%s", message);
    }
  }
};

TrtEngine::TrtEngine(const std::string & engine_path, int device_id)
{
  logger_ = std::make_unique<Logger>();
  try {
    CheckCuda(cudaSetDevice(device_id), "cudaSetDevice");
    CheckCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");
    LoadEngine(engine_path);
  } catch (...) {
    Cleanup();
    throw;
  }
}

TrtEngine::~TrtEngine()
{
  Cleanup();
}

TensorInfo TrtEngine::DescribeTensor(const char * name) const
{
  TensorInfo info;
  info.name = name;
  info.dims = engine_->getTensorShape(name);
  info.dtype = engine_->getTensorDataType(name);
  info.is_input = engine_->getTensorIOMode(name) ==
    nvinfer1::TensorIOMode::kINPUT;
  info.element_count = ElementCount(info.dims, info.name);
  switch (info.dtype) {
    case nvinfer1::DataType::kFLOAT:
      info.byte_size = info.element_count * sizeof(float);
      break;
    case nvinfer1::DataType::kHALF:
      info.byte_size = info.element_count * sizeof(uint16_t);
      break;
    default:
      throw std::runtime_error(
              "tensor \"" + info.name + "\" has unsupported data type");
  }
  return info;
}

void TrtEngine::LoadEngine(const std::string & engine_path)
{
  std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error(
            "cannot open TensorRT engine: " + engine_path +
            " (build it with scripts/build_models.py)");
  }
  const std::streamsize size = file.tellg();
  if (size <= 0) {
    throw std::runtime_error("TensorRT engine is empty: " + engine_path);
  }
  file.seekg(0, std::ios::beg);
  std::vector<char> blob(static_cast<size_t>(size));
  if (!file.read(blob.data(), size)) {
    throw std::runtime_error("cannot read TensorRT engine: " + engine_path);
  }

  runtime_ = nvinfer1::createInferRuntime(*logger_);
  if (runtime_ == nullptr) {
    throw std::runtime_error("createInferRuntime failed");
  }
  engine_ = runtime_->deserializeCudaEngine(blob.data(), blob.size());
  if (engine_ == nullptr) {
    throw std::runtime_error(
            "cannot deserialize TensorRT engine " + engine_path +
            " (was it built on this Jetson with TensorRT 10.3?)");
  }
  context_ = engine_->createExecutionContext();
  if (context_ == nullptr) {
    throw std::runtime_error("createExecutionContext failed");
  }

  bool found_input = false;
  bool found_output = false;
  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    const char * name = engine_->getIOTensorName(i);
    TensorInfo info = DescribeTensor(name);
    if (info.is_input) {
      if (found_input) {
        throw std::runtime_error("engine has more than one input tensor");
      }
      input_info_ = info;
      found_input = true;
    } else if (!found_output) {
      output_info_ = info;
      found_output = true;
    }
  }
  if (!found_input || !found_output) {
    throw std::runtime_error("engine must have one input and one output");
  }
  if (input_info_.dtype != nvinfer1::DataType::kFLOAT ||
    output_info_.dtype != nvinfer1::DataType::kFLOAT)
  {
    throw std::runtime_error(
            "engine I/O must be float32 (rebuild without fp16 I/O formats)");
  }

  CheckCuda(
    cudaMalloc(&device_input_, input_info_.byte_size), "cudaMalloc(input)");
  CheckCuda(
    cudaMalloc(&device_output_, output_info_.byte_size), "cudaMalloc(output)");

  if (!context_->setTensorAddress(input_info_.name.c_str(), device_input_)) {
    throw std::runtime_error("setTensorAddress(input) failed");
  }
  if (!context_->setTensorAddress(output_info_.name.c_str(), device_output_)) {
    throw std::runtime_error("setTensorAddress(output) failed");
  }
}

void TrtEngine::Infer(
  const std::vector<float> & input, std::vector<float> * output)
{
  if (output == nullptr) {
    throw std::runtime_error("Infer: output is null");
  }
  if (input.size() != input_info_.element_count) {
    throw std::runtime_error("Infer: input size does not match the engine");
  }
  output->resize(output_info_.element_count);

  const auto start = std::chrono::steady_clock::now();
  CheckCuda(
    cudaMemcpyAsync(
      device_input_, input.data(), input_info_.byte_size,
      cudaMemcpyHostToDevice, stream_), "cudaMemcpyAsync(input)");

  if (!context_->enqueueV3(stream_)) {
    throw std::runtime_error("TensorRT enqueueV3 failed");
  }

  CheckCuda(
    cudaMemcpyAsync(
      output->data(), device_output_, output_info_.byte_size,
      cudaMemcpyDeviceToHost, stream_), "cudaMemcpyAsync(output)");
  CheckCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

  last_inference_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

void TrtEngine::Cleanup()
{
  if (device_input_ != nullptr) {
    cudaFree(device_input_);
    device_input_ = nullptr;
  }
  if (device_output_ != nullptr) {
    cudaFree(device_output_);
    device_output_ = nullptr;
  }
  if (stream_ != nullptr) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
  delete context_;
  context_ = nullptr;
  delete engine_;
  engine_ = nullptr;
  delete runtime_;
  runtime_ = nullptr;
}

}  // namespace roborts_perception
