// The preconditions: what must hold before the driver talks to the robot (docs/fault_handling.md).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <optional>
#include <string>

#include "fr_robot_driver/preconditions.hpp"

using namespace fr_robot_driver;
using fr_client_library::FaultClass;
using fr_client_library::SoftwareVersion;

namespace
{
const fr_client_library::SoftwareVersion kTarget{3, 8, 4};      // verified
const fr_client_library::SoftwareVersion kSimulator{3, 8, 3};  // verified
const fr_client_library::SoftwareVersion kNewer{3, 8, 5};      // known frame size, not verified
const fr_client_library::SoftwareVersion kTooNew{3, 9, 9};     // no known frame size

// FR5 kinematics, as xacro passes it to the plugin.
const char * kFr5 = "0 0 0 0 0 0; 0 0 0.152 1.5708 0 0; -0.425 0 0 0 0 0; "
  "-0.39501 0 0 0 0 0; 0 0 0.1021 1.5708 0 0; 0 0 0.102 -1.5708 0 0";
constexpr double kFr5Flange = 0.1;

Kinematics fr5()
{
  return *parseKinematics(kFr5, kFr5Flange);
}
}  // namespace

// ---- the software version (sdk_quirks version table) ----

TEST(Preconditions, VerifiedVersionsPass)
{
  EXPECT_TRUE(checkVersion(kTarget, false).ok);
  EXPECT_TRUE(checkVersion(kSimulator, false).ok);
  EXPECT_FALSE(checkVersion(kTarget, false).warn);
}

TEST(Preconditions, AnUnverifiedVersionNeedsPermission)
{
  const auto refused = checkVersion(kNewer, false);
  EXPECT_FALSE(refused.ok);
  EXPECT_NE(refused.message.find("not verified"), std::string::npos);
  const auto allowed = checkVersion(kNewer, true);
  EXPECT_TRUE(allowed.ok);
  EXPECT_TRUE(allowed.warn);
}

TEST(Preconditions, AVersionWithoutAFrameSizeIsAlwaysRefused)
{
  const auto v = checkVersion(kTooNew, true);
  EXPECT_FALSE(v.ok);
  EXPECT_NE(v.message.find("no known state frame size"), std::string::npos);
  const auto unparseable = checkVersion(std::nullopt, true);
  EXPECT_FALSE(unparseable.ok);
}

// ---- the robot model ----

TEST(Preconditions, TheControllerMustBeTheModelThisURDFWasBuiltFor)
{
  EXPECT_TRUE(checkModel("FR5-V1-002(V6.0)", "fairino5_v6", false).ok);
  const auto other = checkModel("FR20-V1-001(V6.0)", "fairino5_v6", false);
  EXPECT_FALSE(other.ok);
  EXPECT_NE(other.message.find("FR20-V1-001(V6.0)"), std::string::npos);
  EXPECT_NE(other.message.find("fairino5_v6"), std::string::npos);
}

TEST(Preconditions, AnUnknownModelNeedsPermissionButStillPasses)
{
  const auto refused = checkModel("FR5-V1-001(V5.0)", "fairino5_v6", false);
  EXPECT_FALSE(refused.ok);
  const auto allowed = checkModel("FR5-V1-001(V5.0)", "fairino5_v6", true);
  EXPECT_TRUE(allowed.ok);
  EXPECT_TRUE(allowed.warn);   // the flange check still guards it
}

// ---- activation ----

TEST(Preconditions, AStaleOrFaultedFrameIsNotActivatable)
{
  fr_client_library::RobotState384 s{};
  EXPECT_FALSE(checkActivatable(s, false, nullptr, 10.0, 1.0, "fairino5_v6").ok);
  s.main_code = 1; s.sub_code = 23;
  const auto fault = checkActivatable(s, true, nullptr, 10.0, 1.0, "fairino5_v6");
  EXPECT_FALSE(fault.ok);
  EXPECT_NE(fault.message.find("sub_code=23"), std::string::npos);
  s.main_code = 0; s.EmergencyStop = 1;
  EXPECT_FALSE(checkActivatable(s, true, nullptr, 10.0, 1.0, "fairino5_v6").ok);
  s.EmergencyStop = 0;
  s.rbtEnableState = 0;
  EXPECT_FALSE(checkActivatable(s, true, nullptr, 10.0, 1.0, "fairino5_v6").ok);
}

TEST(Preconditions, TheFlangeCheckIsPartOfActivation)
{
  const Kinematics k = fr5();
  fr_client_library::RobotState384 s{};
  s.rbtEnableState = 1;
  const std::array<double, 6> q{0.0, -90.0, 90.0, -90.0, -90.0, 0.0};
  std::copy(q.begin(), q.end(), s.jt_cur_pos);
  // the same pose as the controller reports it (fr_description, simulator-verified convention)
  const std::array<double, 6> flange{-497.0, -102.0, 477.0, 180.0, 0.0, 90.0};
  std::copy(flange.begin(), flange.end(), s.flange_cur_pos);
  EXPECT_TRUE(checkActivatable(s, true, &k, 10.0, 1.0, "fairino5_v6").ok);
  s.flange_cur_pos[0] -= 50.0;   // re-installed, or wrong zero offsets
  const auto off = checkActivatable(s, true, &k, 10.0, 1.0, "fairino5_v6");
  EXPECT_FALSE(off.ok);
  EXPECT_NE(off.message.find("flange check failed"), std::string::npos);
  EXPECT_TRUE(checkActivatable(s, true, nullptr, 10.0, 1.0, "fairino5_v6").ok);   // fk_check off
}

// ---- the gripper's activation (G1, G2, G11) ----

TEST(Preconditions, TheGripperActivationRefusals)
{
  fr_client_library::RobotState384 s{};
  s.rbtEnableState = 1;
  const std::array<double, 6> still{};
  EXPECT_FALSE(refuseGripperActivation(false, &s, true, false, false, still, 0.5, false).empty());
  EXPECT_FALSE(refuseGripperActivation(true, nullptr, true, false, false, still, 0.5, false).empty());
  EXPECT_FALSE(refuseGripperActivation(true, &s, false, false, false, still, 0.5, false).empty());
  s.main_code = 1;
  EXPECT_NE(refuseGripperActivation(true, &s, true, false, false, still, 0.5, false).find("fault"),
    std::string::npos);
  s.main_code = 0;
  EXPECT_FALSE(refuseGripperActivation(true, &s, true, true, false, still, 0.5, false).empty());
  EXPECT_FALSE(refuseGripperActivation(true, &s, true, false, true, still, 0.5, false).empty());
  const std::array<double, 6> moving{0, 0, 5.0, 0, 0, 0};
  EXPECT_NE(refuseGripperActivation(true, &s, true, false, false, moving, 0.5, false).find("moving"),
    std::string::npos);
  EXPECT_NE(refuseGripperActivation(true, &s, true, false, false, still, 0.5, true).find("commanded"),
    std::string::npos);
  // a faulted, inactive gripper is activatable: that is what clears the fault (G11)
  s.gripper_fault = 1;
  EXPECT_TRUE(refuseGripperActivation(true, &s, true, false, false, still, 0.5, false).empty());
}

// ---- reset_error (F3, F4) ----

TEST(Preconditions, ResetDecisions)
{
  EXPECT_EQ(decideReset(true, FaultClass::RobotFault), ResetDecision::Allowed);
  EXPECT_EQ(decideReset(true, FaultClass::None), ResetDecision::Allowed);  // harmless then
  EXPECT_EQ(decideReset(true, FaultClass::Estop), ResetDecision::Estop);   // F4: restart instead
  EXPECT_EQ(decideReset(true, FaultClass::Comm), ResetDecision::Comm);
  EXPECT_EQ(decideReset(false, FaultClass::None), ResetDecision::NotConfigured);
  EXPECT_STREQ(describe(ResetDecision::Estop), "emergency stop: restart the controller, a reset is "
    "refused");
}
