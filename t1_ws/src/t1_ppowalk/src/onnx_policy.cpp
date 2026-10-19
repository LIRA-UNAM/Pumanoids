#include "t1_ppowalk/onnx_policy.hpp"

#include <algorithm>
#include <stdexcept>

namespace t1_ppowalk
{

namespace
{

// Replaces dynamic dims (batch) with 1 and returns the element count.
size_t resolve_shape(std::vector<int64_t> & shape)
{
  size_t n = 1;
  for (auto & d : shape) {
    if (d <= 0) {
      d = 1;
    }
    n *= static_cast<size_t>(d);
  }
  return n;
}

}  // namespace

OnnxPolicy::OnnxPolicy(const std::string & model_path, int num_threads)
: env_(ORT_LOGGING_LEVEL_WARNING, "t1_ppowalk_policy"),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(num_threads);
  options.SetInterOpNumThreads(1);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options);

  if (session_->GetInputCount() != 1) {
    throw std::runtime_error(
            "Expected a policy with exactly 1 input, got " +
            std::to_string(session_->GetInputCount()));
  }
  if (session_->GetOutputCount() < 1) {
    throw std::runtime_error("Policy has no outputs");
  }

  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

  // The shape infos are views into the TypeInfo objects: keep those alive.
  const Ort::TypeInfo input_type = session_->GetInputTypeInfo(0);
  const Ort::TypeInfo output_type = session_->GetOutputTypeInfo(0);
  const auto input_info = input_type.GetTensorTypeAndShapeInfo();
  const auto output_info = output_type.GetTensorTypeAndShapeInfo();
  if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
    output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
  {
    throw std::runtime_error("Policy input and output must be float32");
  }
  input_shape_ = input_info.GetShape();
  output_shape_ = output_info.GetShape();
  input_.assign(resolve_shape(input_shape_), 0.0f);
  output_.assign(resolve_shape(output_shape_), 0.0f);

  input_tensor_ = Ort::Value::CreateTensor<float>(
    memory_info_, input_.data(), input_.size(), input_shape_.data(), input_shape_.size());
  output_tensor_ = Ort::Value::CreateTensor<float>(
    memory_info_, output_.data(), output_.size(), output_shape_.data(), output_shape_.size());
}

std::string OnnxPolicy::metadata(const std::string & key) const
{
  Ort::AllocatorWithDefaultOptions allocator;
  const Ort::ModelMetadata meta = session_->GetModelMetadata();
  const auto value = meta.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
  return value ? std::string(value.get()) : std::string();
}

const std::vector<float> & OnnxPolicy::infer(const std::vector<float> & obs)
{
  if (obs.size() != input_.size()) {
    throw std::invalid_argument(
            "Observation size " + std::to_string(obs.size()) + " != policy input size " +
            std::to_string(input_.size()));
  }
  std::copy(obs.begin(), obs.end(), input_.begin());
  const char * input_names[] = {input_name_.c_str()};
  const char * output_names[] = {output_name_.c_str()};
  session_->Run(
    Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);
  return output_;
}

}  // namespace t1_ppowalk
