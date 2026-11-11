#include <gtest/gtest.h>

#include <cmath>

#include "k1_ppowalk/ankle_linkage.hpp"
#include "k1_ppowalk/observation.hpp"
#include "k1_ppowalk/motion.hpp"

TEST(Tilt, FromQuaternion)
{
  const double h = std::sqrt(0.5);
  auto g = k1_ppowalk::projected_gravity(1, 0, 0, 0);
  EXPECT_NEAR(k1_ppowalk::tilt_deg(g[0], g[1], g[2]), 0.0, 1e-9);
  g = k1_ppowalk::projected_gravity(h, h, 0, 0);  // 90 deg roll: lying on the side
  EXPECT_NEAR(k1_ppowalk::tilt_deg(g[0], g[1], g[2]), 90.0, 1e-6);
  g = k1_ppowalk::projected_gravity(h, 0, h, 0);  // 90 deg pitch: lying on the back/front
  EXPECT_NEAR(k1_ppowalk::tilt_deg(g[0], g[1], g[2]), 90.0, 1e-6);
  const double a = 30.0 * M_PI / 180.0;  // 30 deg pitch
  g = k1_ppowalk::projected_gravity(std::cos(a / 2), 0, std::sin(a / 2), 0);
  EXPECT_NEAR(k1_ppowalk::tilt_deg(g[0], g[1], g[2]), 30.0, 1e-6);
  // Yaw does not tilt the robot.
  g = k1_ppowalk::projected_gravity(std::cos(0.6), 0, 0, std::sin(0.6));
  EXPECT_NEAR(k1_ppowalk::tilt_deg(g[0], g[1], g[2]), 0.0, 1e-6);
  g = k1_ppowalk::projected_gravity(0, 1, 0, 0);  // upside down
  EXPECT_NEAR(k1_ppowalk::tilt_deg(g[0], g[1], g[2]), 180.0, 1e-6);
}

TEST(SafeMove, DurationRespectsPeakVelocity)
{
  const double T = k1_ppowalk::safe_move_duration({0.0, 0.0}, {0.6, -0.3}, 0.3, 1.0);
  EXPECT_NEAR(T, 0.5 * M_PI * 0.6 / 0.3, 1e-9);
  EXPECT_DOUBLE_EQ(k1_ppowalk::safe_move_duration({0.0}, {0.01}, 0.3, 2.0), 2.0);
  // Peak speed of the cosine profile equals max_vel.
  const double ds = 1e-6;
  const double v = 0.6 * (k1_ppowalk::smooth_alpha(0.5 + ds) - k1_ppowalk::smooth_alpha(0.5)) / (ds * T);
  EXPECT_NEAR(v, 0.3, 1e-4);
}

TEST(SafeMove, Blend)
{
  EXPECT_DOUBLE_EQ(k1_ppowalk::smooth_alpha(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(k1_ppowalk::smooth_alpha(2.0), 1.0);
  auto q = k1_ppowalk::blend({0.0, 1.0}, {1.0, 3.0}, 0.5);
  EXPECT_DOUBLE_EQ(q[0], 0.5);
  EXPECT_DOUBLE_EQ(q[1], 2.0);
}

TEST(Remote, StickCommand)
{
  // Deadzone: exactly zero, so a released stick means "stand still".
  EXPECT_EQ(k1_ppowalk::stick_command(0.0, 0.1, -0.7, 0.7), 0.0);
  EXPECT_EQ(k1_ppowalk::stick_command(0.09, 0.1, -0.7, 0.7), 0.0);
  EXPECT_EQ(k1_ppowalk::stick_command(-0.1, 0.1, -0.7, 0.7), 0.0);
  // Full deflection reaches the (asymmetric) limits.
  EXPECT_DOUBLE_EQ(k1_ppowalk::stick_command(1.0, 0.1, -0.5, 0.7), 0.7);
  EXPECT_DOUBLE_EQ(k1_ppowalk::stick_command(-1.0, 0.1, -0.5, 0.7), -0.5);
  // Linear between the deadzone and full deflection.
  EXPECT_NEAR(k1_ppowalk::stick_command(0.55, 0.1, -0.7, 0.7), 0.35, 1e-12);
  EXPECT_NEAR(k1_ppowalk::stick_command(-0.55, 0.1, -0.7, 0.7), -0.35, 1e-12);
  // Out of range / invalid input.
  EXPECT_DOUBLE_EQ(k1_ppowalk::stick_command(3.0, 0.1, -0.7, 0.7), 0.7);
  EXPECT_EQ(k1_ppowalk::stick_command(std::nan(""), 0.1, -0.7, 0.7), 0.0);
  // No deadzone; a zero limit blocks that direction.
  EXPECT_DOUBLE_EQ(k1_ppowalk::stick_command(0.5, 0.0, -0.7, 0.7), 0.35);
  EXPECT_EQ(k1_ppowalk::stick_command(-0.8, 0.1, 0.0, 0.7), 0.0);
}

TEST(Imu, QuatFromRpy)
{
  // Upright with any yaw: gravity straight down in the body frame.
  auto q = k1_ppowalk::quat_from_rpy(0.0, 0.0, 1.2);
  auto g = k1_ppowalk::projected_gravity(q[0], q[1], q[2], q[3]);
  EXPECT_NEAR(g[0], 0.0, 1e-12);
  EXPECT_NEAR(g[1], 0.0, 1e-12);
  EXPECT_NEAR(g[2], -1.0, 1e-12);
  // Nose down (+pitch about y): gravity gets a +x body component.
  q = k1_ppowalk::quat_from_rpy(0.0, 0.3, 0.0);
  g = k1_ppowalk::projected_gravity(q[0], q[1], q[2], q[3]);
  EXPECT_NEAR(g[0], std::sin(0.3), 1e-12);
  EXPECT_NEAR(g[2], -std::cos(0.3), 1e-12);
  // +roll about x: gravity gets a -y body component.
  q = k1_ppowalk::quat_from_rpy(0.2, 0.0, 0.0);
  g = k1_ppowalk::projected_gravity(q[0], q[1], q[2], q[3]);
  EXPECT_NEAR(g[1], -std::sin(0.2), 1e-12);
  // Yaw does not change the gravity direction in the body frame.
  const auto a = k1_ppowalk::quat_from_rpy(0.2, 0.1, 0.0);
  const auto b = k1_ppowalk::quat_from_rpy(0.2, 0.1, 2.5);
  const auto ga = k1_ppowalk::projected_gravity(a[0], a[1], a[2], a[3]);
  const auto gb = k1_ppowalk::projected_gravity(b[0], b[1], b[2], b[3]);
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(ga[k], gb[k], 1e-12);
  }
}

// Keyframe linkage poses of k1_parallel_constants.py (solved with mjlab's IK).
TEST(AnkleLinkage, MatchesTrainingKeyframes)
{
  for (bool right : {false, true}) {
    auto c = k1_ppowalk::k1_ankle_crank_ik(-0.4, 0.0, right);  // HOME
    EXPECT_NEAR(c[0], 0.334363, 1e-5);
    EXPECT_NEAR(c[1], 0.317093, 1e-5);
    c = k1_ppowalk::k1_ankle_crank_ik(-0.525, 0.0, right);  // KICK
    EXPECT_NEAR(c[0], 0.430693, 1e-5);
    EXPECT_NEAR(c[1], 0.415008, 1e-5);
    c = k1_ppowalk::k1_ankle_crank_ik(0.03, 0.0, right);  // CRAWL
    EXPECT_NEAR(c[0], -0.026431, 1e-5);
    EXPECT_NEAR(c[1], -0.024624, 1e-5);
  }
}

TEST(AnkleLinkage, RollMovesCranksOppositelyAndMirrors)
{
  const auto l = k1_ppowalk::k1_ankle_crank_ik(0.0, 0.2, false);
  const auto r = k1_ppowalk::k1_ankle_crank_ik(0.0, -0.2, true);
  EXPECT_LT(l[0] * l[1], 0.0);
  EXPECT_NEAR(l[0], r[0], 1e-9);  // mirrored roll on the mirrored leg
  EXPECT_NEAR(l[1], r[1], 1e-9);
}
