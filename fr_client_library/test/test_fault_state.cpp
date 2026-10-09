#include <gtest/gtest.h>

#include "fr_client_library/fault_state.hpp"

using namespace fr_client_library;

namespace
{
RobotState384 healthy()
{
  RobotState384 s{};
  s.rbtEnableState = 1;  // the simulator reports 1 when enabled
  s.robot_state = 1;
  return s;
}
}  // namespace

TEST(FaultState, HealthyFrameIsNone)
{
  EXPECT_EQ(classifyState(healthy()), FaultClass::None);
  EXPECT_EQ(describeFault(healthy()), "no fault");
}

TEST(FaultState, EachTripwireIsARobotFault)
{
  const auto check = [](auto mutate, const char * what) {
      RobotState384 s = healthy();
      mutate(s);
      EXPECT_EQ(classifyState(s), FaultClass::RobotFault) << what;
      EXPECT_NE(describeFault(s), "no fault") << what;
    };
  check([](RobotState384 & s) {s.main_code = 1;}, "main_code");
  check([](RobotState384 & s) {s.main_code = -3;}, "negative main_code");
  check([](RobotState384 & s) {s.collisionState = 1;}, "collision");
  check([](RobotState384 & s) {s.safety_stop0_state = 1;}, "SI0");
  check([](RobotState384 & s) {s.safety_stop1_state = 1;}, "SI1");
  check([](RobotState384 & s) {s.rbtEnableState = 0;}, "disabled");
  check([](RobotState384 & s) {s.robot_state = 4;}, "drag teaching");
}

TEST(FaultState, SubCodeAloneIsNotAFault)  // main_code is what the doc keys on
{
  RobotState384 s = healthy();
  s.sub_code = 23;
  EXPECT_EQ(classifyState(s), FaultClass::None);
}

TEST(FaultState, EmergencyStopOutranksEverythingElse)
{
  RobotState384 s = healthy();
  s.EmergencyStop = 1;
  EXPECT_EQ(classifyState(s), FaultClass::Estop);
  s.main_code = 1;
  s.collisionState = 1;
  EXPECT_EQ(classifyState(s), FaultClass::Estop);
  EXPECT_NE(describeFault(s).find("emergency stop"), std::string::npos);
}

TEST(FaultState, DescriptionNamesTheCodes)
{
  RobotState384 s = healthy();
  s.main_code = 1;
  s.sub_code = 23;
  EXPECT_EQ(describeFault(s), "main_code=1 sub_code=23");  // as observed on the simulator (S11)
  s.collisionState = 1;
  EXPECT_EQ(describeFault(s), "main_code=1 sub_code=23, collision");
}

TEST(FaultState, SeverityOrdersAsTheDocSays)
{
  EXPECT_LT(static_cast<int>(FaultClass::None), static_cast<int>(FaultClass::RobotFault));
  EXPECT_LT(static_cast<int>(FaultClass::RobotFault), static_cast<int>(FaultClass::Estop));
  EXPECT_LT(static_cast<int>(FaultClass::Estop), static_cast<int>(FaultClass::Comm));
  EXPECT_STREQ(toString(FaultClass::Comm), "communication fault");
}

TEST(FaultState, QueueLengthIsTheStateFrameField)
{
  RobotState384 s = healthy();
  s.mc_queue_len = 42;
  EXPECT_EQ(servoQueueLength(s), 42);
}
