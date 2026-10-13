// The status the driver reports (`~/status`): what it says in each situation, since when, and
// which fault came before.
#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "fr_robot_driver/status_report.hpp"

using namespace fr_robot_driver;
using diagnostic_msgs::msg::DiagnosticStatus;
using std::chrono::seconds;

namespace
{
std::string value(const DiagnosticStatus & s, const std::string & key)
{
  for (const auto & kv : s.values) {
    if (kv.key == key) {return kv.value;}
  }
  return "<missing>";
}

StatusInput healthy(HardwareState hw = HardwareState::Active)
{
  StatusInput in;
  in.hardware = hw;
  in.link_up = true;
  in.frame.rbtEnableState = 1;
  in.frame.robot_state = 1;
  in.frame_age_ms = 3;
  return in;
}

const auto t0 = std::chrono::system_clock::from_time_t(1760000000);  // 2025-10-09T08:53:20Z
}  // namespace

TEST(StatusReport, HealthyIsOk)
{
  StatusTracker t;
  t.update(healthy(), t0);
  const auto & s = t.status();
  EXPECT_EQ(s.level, DiagnosticStatus::OK);
  EXPECT_EQ(s.message, "ok");
  EXPECT_EQ(value(s, "fault_class"), "0");
  EXPECT_EQ(value(s, "hardware_state"), "active");
  EXPECT_EQ(value(s, "link"), "up");
  EXPECT_EQ(value(s, "since"), "2025-10-09T08:53:20.000Z");
  EXPECT_EQ(value(s, "previous"), "none");
}

TEST(StatusReport, UnconfiguredWithoutAConnectionIsAWarningNotAFault)
{
  StatusTracker t;
  t.update(StatusInput{}, t0);
  EXPECT_EQ(t.status().level, DiagnosticStatus::WARN);
  EXPECT_EQ(value(t.status(), "fault_class"), "0");
  EXPECT_EQ(value(t.status(), "link"), "down");
  EXPECT_EQ(value(t.status(), "main_code"), "unknown");
  EXPECT_EQ(value(t.status(), "last_frame_age_ms"), "none");
}

TEST(StatusReport, TheFrameAgeAloneIsNotAChange)
{
  StatusTracker t;
  auto in = healthy();
  EXPECT_TRUE(t.update(in, t0));
  in.frame_age_ms = 7;
  EXPECT_FALSE(t.update(in, t0 + seconds(1)));
  EXPECT_EQ(value(t.status(), "last_frame_age_ms"), "7");
  in.frame.main_code = 1;
  EXPECT_TRUE(t.update(in, t0 + seconds(2)));
}

TEST(StatusReport, ALostLinkIsReportedAsInterruptedNotAsACable)
{
  StatusTracker t;
  StatusInput in;
  in.hardware = HardwareState::Unconfigured;
  in.comm = CommReason::NoFrame;
  in.frame_age_ms = 4000;
  t.update(in, t0);
  const auto & s = t.status();
  EXPECT_EQ(s.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(value(s, "fault_class"), "3");
  EXPECT_NE(s.message.find("communication fault: no state frame in time"), std::string::npos) << s.message;
  EXPECT_NE(s.message.find("interrupted"), std::string::npos);
  EXPECT_NE(s.message.find("emergency stop"), std::string::npos);   // the controller may have lost power
  EXPECT_EQ(value(s, "last_frame_age_ms"), "4000");
}

TEST(StatusReport, LinkLossThenTheControllersOwnFaultKeepsTheHistory)
{
  StatusTracker t;
  t.update(healthy(), t0);

  StatusInput down;
  down.hardware = HardwareState::Unconfigured;
  down.comm = CommReason::NoFrame;
  t.update(down, t0 + seconds(10));
  EXPECT_EQ(value(t.status(), "fault_class"), "3");
  EXPECT_EQ(value(t.status(), "since"), "2025-10-09T08:53:30.000Z");
  EXPECT_TRUE(t.update(down, t0 + seconds(40)) == false);                // still the same condition
  EXPECT_EQ(value(t.status(), "since"), "2025-10-09T08:53:30.000Z");

  auto back = healthy(HardwareState::Unconfigured);                    // the controller is online again
  back.comm = CommReason::NoFrame;
  back.frame.main_code = 1;
  back.frame.sub_code = 149;
  back.frame.safety_stop0_state = 1;
  back.frame.rbtEnableState = 0;
  EXPECT_TRUE(t.update(back, t0 + seconds(70)));
  const auto & s = t.status();
  EXPECT_EQ(s.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(value(s, "fault_class"), "1");
  EXPECT_EQ(value(s, "main_code"), "1");
  EXPECT_EQ(value(s, "sub_code"), "149");
  EXPECT_EQ(value(s, "hardware_state"), "unconfigured");
  EXPECT_EQ(value(s, "link"), "up");
  EXPECT_NE(s.message.find("robot fault: main_code=1 sub_code=149"), std::string::npos) << s.message;
  EXPECT_EQ(value(s, "since"), "2025-10-09T08:54:30.000Z");
  const std::string prev = value(s, "previous");
  EXPECT_NE(prev.find("communication fault"), std::string::npos) << prev;
  EXPECT_NE(prev.find("from 2025-10-09T08:53:30.000Z to 2025-10-09T08:54:30.000Z"), std::string::npos) << prev;
}

TEST(StatusReport, AnOnlineControllerWithUnconfiguredHardwareAsksForConfigure)
{
  StatusTracker t;
  auto in = healthy(HardwareState::Unconfigured);
  in.comm = CommReason::NoFrame;
  t.update(in, t0);
  EXPECT_EQ(t.status().level, DiagnosticStatus::WARN);
  EXPECT_EQ(value(t.status(), "fault_class"), "0");
  EXPECT_NE(t.status().message.find("configure and activate"), std::string::npos);
}

TEST(StatusReport, EmergencyStopStaysDistinctFromARobotFault)
{
  StatusTracker t;
  auto in = healthy(HardwareState::Inactive);
  in.frame.EmergencyStop = 1;
  t.update(in, t0);
  EXPECT_EQ(value(t.status(), "fault_class"), "2");
  EXPECT_EQ(value(t.status(), "estop"), "1");
}

TEST(StatusReport, OurOwnProtectiveStopIsARobotFaultWithItsReason)
{
  StatusTracker t;
  auto in = healthy(HardwareState::Inactive);
  in.protective_stop = ProtectiveStop::StepLimit;
  t.update(in, t0);
  EXPECT_EQ(value(t.status(), "fault_class"), "1");
  EXPECT_EQ(value(t.status(), "main_code"), "0");
  EXPECT_NE(t.status().message.find("step check"), std::string::npos);
  in.protective_stop = ProtectiveStop::HoldTimeout;
  t.update(in, t0);
  EXPECT_EQ(value(t.status(), "fault_class"), "1");
  EXPECT_NE(t.status().message.find("gripper finished"), std::string::npos) << t.status().message;
}

TEST(StatusReport, AServoStreamFailureIsACommunicationFaultEvenWithFrames)
{
  StatusTracker t;
  auto in = healthy();
  in.comm = CommReason::ServoFailed;
  t.update(in, t0);
  EXPECT_EQ(value(t.status(), "fault_class"), "3");
  EXPECT_NE(t.status().message.find("servo stream"), std::string::npos);
}
