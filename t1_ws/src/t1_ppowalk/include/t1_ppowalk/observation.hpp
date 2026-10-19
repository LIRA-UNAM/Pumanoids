#pragma once

#include <array>
#include <string>
#include <vector>

namespace t1_ppowalk
{

// Gravity direction (0, 0, -1) expressed in the body frame, given the body
// orientation quaternion (w, x, y, z). Matches mjlab's projected_gravity_b
// (quat_apply_inverse of the world gravity direction).
std::array<double, 3> projected_gravity(double w, double x, double y, double z);

// Latest robot state, joints already in policy order.
struct RobotState
{
  std::array<double, 3> base_ang_vel{0.0, 0.0, 0.0};
  std::array<double, 4> base_quat{1.0, 0.0, 0.0, 0.0};  // w, x, y, z
  std::vector<double> joint_pos;
  std::vector<double> joint_vel;
  std::array<double, 3> command{0.0, 0.0, 0.0};  // velocity command (vx, vy, wz)
};

// Builds the flat actor observation from an ordered list of mjlab observation
// term names. Supported terms (names as in the mjlab task config):
//   base_ang_vel (3), projected_gravity (3), joint_pos (N, relative to the
//   default pose), joint_vel (N), actions (A, last raw policy output; A
//   defaults to N), command (3, velocity command vx, vy, wz).
class ObservationBuilder
{
public:
  ObservationBuilder(
    std::vector<std::string> terms, std::vector<double> default_joint_pos,
    std::vector<double> term_scales = {}, size_t num_actions = 0);

  size_t size() const {return size_;}
  size_t num_joints() const {return default_joint_pos_.size();}
  size_t num_actions() const {return num_actions_;}
  const std::vector<std::string> & terms() const {return terms_;}

  // Writes the observation into obs (resized to size()).
  void build(
    const RobotState & state, const std::vector<float> & last_action,
    std::vector<float> & obs) const;

private:
  size_t term_size(const std::string & term) const;

  std::vector<std::string> terms_;
  std::vector<double> default_joint_pos_;
  std::vector<double> term_scales_;
  size_t num_actions_{0};
  size_t size_{0};
};

}  // namespace t1_ppowalk
