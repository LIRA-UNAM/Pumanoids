// Closed-form inverse kinematics of the Booster K1 parallel ankle linkage.
//
// Each ankle (pitch about y, then roll about x) is driven by two cranks A and B
// rotating about y on the shank; a rod of fixed length joins each crank tip to
// an anchor on the foot. Geometry from mjlab-biped-playground
// asset_zoo/robots/k1/xmls/k1_parallel.xml (Left_Ankle_Crank_{A,B},
// Left_Ankle_Rod_{A,B}, left_ankle_{a,b}_anchor; the right leg is the mirror
// image in y). The training policy observes and commands the crank angles
// (*_Ankle_A / *_Ankle_B); this IK maps the firmware's serial ankle pitch/roll
// to the crank angles the simulation would have, to check that the robot's
// crank convention (zero, sign, A/B order) matches the training model.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace k1_ppowalk
{

struct AnkleCrankGeometry
{
  std::array<double, 3> pivot;   // crank pivot in the shank frame (left leg)
  std::array<double, 2> arm;     // crank tip in the crank frame (x, z), y = 0
  double rod_length;             // crank tip -> foot anchor
  std::array<double, 3> anchor;  // anchor in the foot frame (left leg)
};

// Left leg; the right leg mirrors every y coordinate.
inline const AnkleCrankGeometry & k1_crank_geometry(int crank)  // 0 = A, 1 = B
{
  static const AnkleCrankGeometry geometry[2] = {
    {{-0.00148194, 0.0242, -0.081637428}, {0.0408490153, -0.00351538739},
      std::hypot(0.0681700153, 0.142537185), {-0.029, 0.024, 0.0175}},
    {{-0.00967794, -0.0238, -0.140057167}, {0.032999524, -0.0243316957},
      std::hypot(0.052124524, 0.0633011373), {-0.029, -0.024, 0.0175}},
  };
  return geometry[crank];
}

// Ankle cross (pitch/roll joint centre) in the shank frame, left leg.
constexpr std::array<double, 3> kK1AnkleCentre{0.00019706, 0.0002, -0.24519};

// Crank angles (A, B) for an ankle pitch/roll. `right` mirrors the geometry.
// The crank equation has two solutions; the one closer to `reference`
// (default: the HOME_KEYFRAME cranks) is returned.
inline std::array<double, 2> k1_ankle_crank_ik(
  double pitch, double roll, bool right,
  const std::array<double, 2> & reference = {0.334363, 0.317093})
{
  const double side = right ? -1.0 : 1.0;
  std::array<double, 2> out{};
  for (int k = 0; k < 2; ++k) {
    const auto & g = k1_crank_geometry(k);
    // Anchor in the shank frame: centre + Ry(pitch) * Rx(roll) * anchor.
    const double ax = g.anchor[0], ay = side * g.anchor[1], az = g.anchor[2];
    const double y1 = ay * std::cos(roll) - az * std::sin(roll);
    const double z1 = ay * std::sin(roll) + az * std::cos(roll);
    const double x2 = ax * std::cos(pitch) + z1 * std::sin(pitch);
    const double z2 = -ax * std::sin(pitch) + z1 * std::cos(pitch);
    const double dx = kK1AnkleCentre[0] + x2 - g.pivot[0];
    const double dy = side * kK1AnkleCentre[1] + y1 - side * g.pivot[1];
    const double dz = kK1AnkleCentre[2] + z2 - g.pivot[2];
    // |d - Ry(t) c| = L  <=>  a cos t + b sin t = K.
    const double cx = g.arm[0], cz = g.arm[1];
    const double K =
      0.5 * (dx * dx + dy * dy + dz * dz + cx * cx + cz * cz - g.rod_length * g.rod_length);
    const double a = dx * cx + dz * cz;
    const double b = dx * cz - dz * cx;
    const double phi = std::atan2(b, a);
    const double delta = std::acos(std::clamp(K / std::hypot(a, b), -1.0, 1.0));
    double t1 = std::remainder(phi + delta, 2.0 * M_PI);
    double t2 = std::remainder(phi - delta, 2.0 * M_PI);
    out[k] = std::abs(t1 - reference[k]) <= std::abs(t2 - reference[k]) ? t1 : t2;
  }
  return out;
}

}  // namespace k1_ppowalk
