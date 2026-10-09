#include <gtest/gtest.h>

#include "fr_client_library/robot_state_384.hpp"

using namespace fr_client_library;

// Layout is enforced by static_assert in the header; this checks the frame arithmetic.
TEST(RobotState384, FrameLengthMatchesDataLen)
{
  EXPECT_EQ(kRobotState384Size - kFrameOverhead, 953u);
}
