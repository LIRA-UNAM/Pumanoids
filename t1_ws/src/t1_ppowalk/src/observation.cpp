#include "t1_ppowalk/observation.hpp"

#include <stdexcept>
#include <utility>

namespace t1_ppowalk
{

std::array<double, 3> projected_gravity(double w, double x, double y, double z)
{
  return {
    2.0 * (w * y - x * z),
    -2.0 * (w * x + y * z),
    -1.0 + 2.0 * (x * x + y * y)};
}

ObservationBuilder::ObservationBuilder(
  std::vector<std::string> terms, std::vector<double> default_joint_pos,
  std::vector<double> term_scales, size_t num_actions)
: terms_(std::move(terms)),
  default_joint_pos_(std::move(default_joint_pos)),
  term_scales_(std::move(term_scales)),
  num_actions_(num_actions > 0 ? num_actions : default_joint_pos_.size())
{
  if (term_scales_.empty()) {
    term_scales_.assign(terms_.size(), 1.0);
  }
  if (term_scales_.size() != terms_.size()) {
    throw std::invalid_argument("observation term scales must match the number of terms");
  }
  for (const auto & term : terms_) {
    size_ += term_size(term);
  }
}

size_t ObservationBuilder::term_size(const std::string & term) const
{
  if (term == "base_ang_vel" || term == "projected_gravity" || term == "command") {
    return 3;
  }
  if (term == "joint_pos" || term == "joint_vel") {
    return default_joint_pos_.size();
  }
  if (term == "actions") {
    return num_actions_;
  }
  throw std::invalid_argument("Unsupported observation term: " + term);
}

void ObservationBuilder::build(
  const RobotState & state, const std::vector<float> & last_action,
  std::vector<float> & obs) const
{
  const size_t n = default_joint_pos_.size();
  if (state.joint_pos.size() != n || state.joint_vel.size() != n) {
    throw std::invalid_argument("joint state size does not match default_joint_pos");
  }
  if (last_action.size() != num_actions_) {
    throw std::invalid_argument("last action size does not match num_actions");
  }
  obs.resize(size_);
  size_t k = 0;
  for (size_t t = 0; t < terms_.size(); ++t) {
    const auto & term = terms_[t];
    const double s = term_scales_[t];
    if (term == "base_ang_vel") {
      for (double v : state.base_ang_vel) {
        obs[k++] = static_cast<float>(s * v);
      }
    } else if (term == "projected_gravity") {
      const auto & q = state.base_quat;
      for (double v : projected_gravity(q[0], q[1], q[2], q[3])) {
        obs[k++] = static_cast<float>(s * v);
      }
    } else if (term == "joint_pos") {
      for (size_t i = 0; i < n; ++i) {
        obs[k++] = static_cast<float>(s * (state.joint_pos[i] - default_joint_pos_[i]));
      }
    } else if (term == "joint_vel") {
      for (size_t i = 0; i < n; ++i) {
        obs[k++] = static_cast<float>(s * state.joint_vel[i]);
      }
    } else if (term == "actions") {
      for (size_t i = 0; i < num_actions_; ++i) {
        obs[k++] = static_cast<float>(s * last_action[i]);
      }
    } else if (term == "command") {
      for (double v : state.command) {
        obs[k++] = static_cast<float>(s * v);
      }
    }
  }
}

}  // namespace t1_ppowalk
