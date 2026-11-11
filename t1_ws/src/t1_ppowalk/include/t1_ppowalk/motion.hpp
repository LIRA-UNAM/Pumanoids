#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace t1_ppowalk
{

// Tilt of the body z axis from vertical (degrees), from the gravity direction
// in the body frame (t1_ppowalk::projected_gravity): 0 when upright, 180 upside down.
inline double tilt_deg(double gx, double gy, double gz)
{
  const double norm = std::sqrt(gx * gx + gy * gy + gz * gz);
  if (norm < 1e-9) {
    return 180.0;
  }
  const double c = std::clamp(-gz / norm, -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

// Orientation quaternion (w, x, y, z) from the Booster IMU Euler angles
// (roll, pitch, yaw; intrinsic Z-Y-X: q = qz(yaw) * qy(pitch) * qx(roll)).
inline std::array<double, 4> quat_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(0.5 * roll), sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch), sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw), sy = std::sin(0.5 * yaw);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

// Duration of a cosine-profile move whose peak joint speed is <= max_vel
// (peak = pi/2 * average speed), never shorter than min_duration.
inline double safe_move_duration(
  const std::vector<double> & from, const std::vector<double> & to, double max_vel,
  double min_duration)
{
  double delta = 0.0;
  for (size_t i = 0; i < from.size() && i < to.size(); ++i) {
    delta = std::max(delta, std::abs(to[i] - from[i]));
  }
  return std::max(min_duration, 0.5 * M_PI * delta / std::max(max_vel, 1e-6));
}

// Cosine blend weight for s in [0, 1] (zero velocity at both ends).
inline double smooth_alpha(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(M_PI * s);
}

// Remote stick deflection s in [-1, 1] -> command in [lo, hi] (lo <= 0 <= hi).
// Inside the deadzone the command is exactly 0 (standing still); beyond it the
// deflection is rescaled so that full deflection reaches the limit.
inline double stick_command(double s, double deadzone, double lo, double hi)
{
  if (!std::isfinite(s)) {
    return 0.0;
  }
  s = std::clamp(s, -1.0, 1.0);
  const double mag = std::abs(s);
  if (mag <= deadzone) {
    return 0.0;
  }
  const double x = (mag - deadzone) / std::max(1.0 - deadzone, 1e-6);
  return s > 0.0 ? x * hi : -x * std::abs(lo);
}

inline std::vector<double> blend(
  const std::vector<double> & from, const std::vector<double> & to, double alpha)
{
  std::vector<double> out(from.size());
  for (size_t i = 0; i < from.size(); ++i) {
    out[i] = from[i] + alpha * (to[i] - from[i]);
  }
  return out;
}

}  // namespace t1_ppowalk
