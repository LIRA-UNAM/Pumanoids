#pragma once

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <vector>

namespace t1_ppowalk
{

// Thin wrapper around an ONNX Runtime session for an MLP policy with a single
// [1, num_obs] float input and a [1, num_actions] float output. Input/output
// names and sizes are read from the model.
class OnnxPolicy
{
public:
  explicit OnnxPolicy(const std::string & model_path, int num_threads = 1);

  size_t num_obs() const {return input_.size();}
  size_t num_actions() const {return output_.size();}
  const std::string & input_name() const {return input_name_;}
  const std::string & output_name() const {return output_name_;}
  // Custom metadata entry of the model (e.g. "joint_names"), empty if absent.
  std::string metadata(const std::string & key) const;

  // Runs inference. obs.size() must equal num_obs(). The returned reference is
  // valid until the next call.
  const std::vector<float> & infer(const std::vector<float> & obs);

private:
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo memory_info_;
  std::string input_name_;
  std::string output_name_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
  std::vector<float> input_;
  std::vector<float> output_;
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
};

}  // namespace t1_ppowalk
