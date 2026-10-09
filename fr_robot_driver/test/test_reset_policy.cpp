#include <gtest/gtest.h>

#include "fr_robot_driver/reset_policy.hpp"

using fr_client_library::FaultClass;
using namespace fr_robot_driver;

TEST(ResetPolicy, AllowedWithOrWithoutARobotFault)
{
  EXPECT_EQ(decideReset(true, FaultClass::RobotFault), ResetDecision::Allowed);
  EXPECT_EQ(decideReset(true, FaultClass::None), ResetDecision::Allowed);  // harmless then
}

TEST(ResetPolicy, EmergencyStopIsNeverReset)  // F4: only a controller restart clears it
{
  EXPECT_EQ(decideReset(true, FaultClass::Estop), ResetDecision::Estop);
}

TEST(ResetPolicy, NoResetOnAnUntrustworthyLinkOrWithoutConnection)
{
  EXPECT_EQ(decideReset(true, FaultClass::Comm), ResetDecision::Comm);
  EXPECT_EQ(decideReset(false, FaultClass::None), ResetDecision::NotConfigured);
  EXPECT_EQ(decideReset(false, FaultClass::Estop), ResetDecision::NotConfigured);
}

TEST(ResetPolicy, EveryDecisionHasAMessage)
{
  for (auto d : {ResetDecision::Allowed, ResetDecision::NotConfigured, ResetDecision::Estop,
      ResetDecision::Comm})
  {
    EXPECT_STRNE(describe(d), "?");
  }
}
