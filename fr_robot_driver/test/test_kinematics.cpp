#include <gtest/gtest.h>

#include <vector>

#include "fr_robot_driver/kinematics.hpp"

using namespace fr_robot_driver;

namespace
{
// fairino5_v6 default_kinematics.yaml (fr_description), as xacro passes it to the plugin
const char * kFr5 = "0 0 0 0 0 0; 0 0 0.152 1.5708 0 0; -0.425 0 0 0 0 0; "
  "-0.39501 0 0 0 0 0; 0 0 0.1021 1.5708 0 0; 0 0 0.102 -1.5708 0 0";
constexpr double kFr5Flange = 0.1;
// fairino20_v6, a bigger robot
const char * kFr20 = "0 0 0 0 0 0; 0 0 0.215 1.5708 0 0; -1 0 0 0 0 0; "
  "-0.716 0 0 0 0 0; 0 0 0.16601 1.5708 0 0; 0 0 0.138 -1.5708 0 0";
constexpr double kFr20Flange = 0.12;

struct Sample
{
  std::array<double, 6> q;  // jt_cur_pos [deg]
  double flange[6];         // flange_cur_pos [mm, deg]
};

// Recorded from the 3.8.3 simulator (FR5-V1-002(V6.0)): both fields of the same state frame.
const std::vector<Sample> kSimulator = {
  {{0, -90, 90, -90, -90, 0}, {-497.0000, -102.0000, 477.0000, 180.0000, 0.0000, 90.0000}},
  {{29.225, -70.517, 60.775, -80.258, -99.742, 19.483}, {-510.8683, -383.5957, 520.9591, -176.7174, -9.1858, 99.4862}},
  {{-44.150, -109.547, 99.547, -60.227, -80.227, -29.433}, {-304.7836, 130.2283, 493.8147, 166.8824, 17.7927, 71.5038}},
  {{59.125, -60.417, 40.500, -119.500, -89.916, 44.375}, {-283.3486, -673.5660, 668.5759, -140.1826, 32.1467, 116.7092}},
  {{0.875, -89.562, 89.271, -90.437, -90.000, 89.344}, {-499.8082, -109.4263, 480.2093, -179.9921, 0.7081, 1.4877}},
  {{89.250, -80.083, 70.167, -99.917, -60.250, 0.750}, {144.1846, -589.5874, 591.6464, -157.7886, 28.1085, -170.5031}},
};
}  // namespace

TEST(Kinematics, ParsesWhatXacroPasses)
{
  const auto k = parseKinematics(kFr5, kFr5Flange);
  ASSERT_TRUE(k.has_value());
  EXPECT_DOUBLE_EQ(k->joints[1].xyz[2], 0.152);
  EXPECT_DOUBLE_EQ(k->joints[5].rpy[0], -1.5708);
  EXPECT_DOUBLE_EQ(k->flange_offset, 0.1);
}

TEST(Kinematics, RejectsMalformedParameters)
{
  EXPECT_FALSE(parseKinematics("", 0.1).has_value());
  EXPECT_FALSE(parseKinematics("0 0 0 0 0 0; 0 0 0 0 0 0", 0.1).has_value());                       // 2 joints
  EXPECT_FALSE(parseKinematics("0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0", 0.1).has_value());
  EXPECT_FALSE(parseKinematics("0 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0", 0.1).has_value());
  EXPECT_FALSE(parseKinematics("a b c d e f; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0", 0.1).has_value());
  EXPECT_FALSE(parseKinematics(std::string(kFr5) + "; 0 0 0 0 0 0", 0.1).has_value());              // 7 joints
}

// The convention (mm/deg, URDF base_link, URDF rpy, flange = wrist_3 + flange_offset) as measured.
TEST(Kinematics, ReproducesTheSimulatorsFlangePoseAtSixPoses)
{
  const auto k = parseKinematics(kFr5, kFr5Flange);
  ASSERT_TRUE(k.has_value());
  for (const auto & s : kSimulator) {
    const auto m = compareFlange(*k, s.q, s.flange);
    EXPECT_LT(m.position_mm, 0.5) << s.q[0] << " " << s.q[1];   // measured: <= 0.25 mm
    EXPECT_LT(m.angle_deg, 0.2) << s.q[0] << " " << s.q[1];     // measured: <= 0.1 deg
  }
}

TEST(Kinematics, AWrongModelIsFarOff)
{
  const auto fr20 = parseKinematics(kFr20, kFr20Flange);
  ASSERT_TRUE(fr20.has_value());
  for (const auto & s : kSimulator) {
    EXPECT_GT(compareFlange(*fr20, s.q, s.flange).position_mm, 100.0);   // FR20 is a lot bigger
  }
}

TEST(Kinematics, ZeroOffsetsAndSignConventionsAreCaught)
{
  const auto k = parseKinematics(kFr5, kFr5Flange);
  ASSERT_TRUE(k.has_value());
  auto offset = kSimulator[1];
  offset.q[1] += 5.0;                      // joint 2 zero off by 5 deg
  EXPECT_GT(compareFlange(*k, offset.q, offset.flange).position_mm, 10.0);
  auto flipped = kSimulator[1];
  flipped.q[0] = -flipped.q[0];            // joint 1 turning the other way
  EXPECT_GT(compareFlange(*k, flipped.q, flipped.flange).position_mm, 10.0);
  auto wrist = kSimulator[3];
  wrist.q[5] += 3.0;                       // only the last joint: position unchanged, orientation not
  const auto m = compareFlange(*k, wrist.q, wrist.flange);
  EXPECT_LT(m.position_mm, 0.5);
  EXPECT_NEAR(m.angle_deg, 3.0, 0.3);
}

TEST(Kinematics, EquivalentEulerAnglesCompareEqual)
{
  const auto k = parseKinematics(kFr5, kFr5Flange);
  ASSERT_TRUE(k.has_value());
  const auto s = kSimulator[0];            // rx = +180
  double wrapped[6];
  std::copy(std::begin(s.flange), std::end(s.flange), wrapped);
  wrapped[3] = -180.0;                     // the same orientation written the other way
  EXPECT_LT(compareFlange(*k, s.q, wrapped).angle_deg, 0.2);
  // pitch near +-90 deg (gimbal lock): different roll/yaw split, one rotation
  const std::array<double, 6> q{0, 0, 0, 0, 0, 0};
  double a[6] = {0, 0, 0, 10.0, 90.0, 20.0};
  double b[6] = {0, 0, 0, 20.0, 90.0, 30.0};   // at pitch +90 only (yaw - roll) matters: 10 in both
  const auto ma = compareFlange(*k, q, a);
  const auto mb = compareFlange(*k, q, b);
  EXPECT_NEAR(ma.angle_deg, mb.angle_deg, 1e-6);
}

TEST(Kinematics, IdentityRotationAtTheOrigin)
{
  Kinematics k;                             // all zeros: the flange sits at the origin
  const double vendor[6] = {0, 0, 0, 0, 0, 0};
  const auto m = compareFlange(k, {0, 0, 0, 0, 0, 0}, vendor);
  EXPECT_NEAR(m.position_mm, 0.0, 1e-9);
  EXPECT_NEAR(m.angle_deg, 0.0, 1e-6);
}
