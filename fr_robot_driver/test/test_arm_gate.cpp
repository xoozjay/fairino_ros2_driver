// ArmGate: the write-side rules without a plugin, sockets or wall time (sdk_quirks S11, S14, G9).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "fr_robot_driver/arm_gate.hpp"

using namespace fr_robot_driver;
using fr_client_library::JointArray;

namespace
{
constexpr double kDeg = M_PI / 180.0;

// FR5: the URDF's fastest joint is 3.15 rad/s = 180 deg/s, so 1.44 deg per 8 ms step.
std::array<double, 6> fr5Steps() {return {1.44, 1.44, 1.44, 1.44, 1.44, 1.44};}

JointArray at(double first, double rest = 0.0)
{
  return JointArray{first, rest, rest, rest, rest, rest};
}
}  // namespace

TEST(ArmGate, TheInitialPositionIsTheLastCommand)
{
  ArmGate g(1, at(12.5), fr5Steps());
  EXPECT_EQ(g.lastCommand()[0], 12.5);
  EXPECT_FALSE(g.commandedMoved(at(12.5)));
  EXPECT_TRUE(g.commandedMoved(at(13.0)));
}

TEST(ArmGate, ACommandWithinTheStepIsSent)
{
  ArmGate g(1, at(0.0), fr5Steps());
  const auto d = g.decide(at(1.44), false);
  EXPECT_EQ(d.action, Segment::Send);
  EXPECT_DOUBLE_EQ(d.target[0], 1.44);
  g.sent(d.target);
  EXPECT_EQ(g.lastCommand()[0], 1.44);
}

TEST(ArmGate, ACommandBeyondTheStepIsRefusedAndSaysWhichJoint)
{
  ArmGate g(1, at(0.0), fr5Steps());
  const auto d = g.decide(at(1.45), false);
  EXPECT_EQ(d.action, Segment::Refuse);
  EXPECT_EQ(d.violation.joint, 0);
  EXPECT_DOUBLE_EQ(d.violation.from_deg, 0.0);
  EXPECT_DOUBLE_EQ(d.violation.to_deg, 1.45);
  EXPECT_DOUBLE_EQ(d.violation.limit_deg, 1.44);
}

TEST(ArmGate, ANonFiniteCommandIsRefused)
{
  ArmGate g(1, at(0.0), fr5Steps());
  EXPECT_EQ(g.decide(at(std::nan("")), false).action, Segment::Refuse);
  EXPECT_EQ(g.decide(at(1.0, std::numeric_limits<double>::infinity()), false).action,
    Segment::Refuse);
}

TEST(ArmGate, ASmallerJointHasASmallerStep)
{
  std::array<double, 6> steps{1.44, 1.44, 1.44, 0.5, 0.5, 0.5};
  ArmGate g(1, at(0.0), steps);
  EXPECT_EQ(g.decide(at(0.6), false).action, Segment::Send);         // joint 1: 1.44 deg allowed
  JointArray t = at(0.0);
  t[3] = 0.6;                                                       // joint 4: only 0.5 deg
  const auto d = g.decide(t, false);
  EXPECT_EQ(d.action, Segment::Refuse);
  EXPECT_EQ(d.violation.joint, 3);
}

TEST(ArmGate, WhileTheGripperMovesTheArmStaysWhereItIs)
{
  ArmGate g(1, at(10.0), fr5Steps());
  const auto d = g.decide(at(30.0), true);   // a jump that would be refused while moving
  EXPECT_EQ(d.action, Segment::Send);
  EXPECT_DOUBLE_EQ(d.target[0], 10.0);       // the parked position, not the command
  g.sent(d.target);
  EXPECT_EQ(g.lastCommand()[0], 10.0);
  // The controller still asks for 30 deg: the gripper does not start (the arm goes first).
  EXPECT_TRUE(g.commandedMoved(at(30.0)));
  // A controller that pauses its trajectory keeps its command within a cycle of the parked
  // position; then the gripper may start.
  EXPECT_FALSE(g.commandedMoved(at(10.0)));
}

TEST(ArmGate, OnlyEveryRatioThCycleIsASegment)
{
  ArmGate g(4, at(0.0), fr5Steps());         // 500 Hz controller, cmdT 8 ms
  int segments = 0;
  for (int i = 0; i < 400; ++i) {
    if (g.sendCycle()) {++segments;}
  }
  EXPECT_EQ(segments, 100);
  EXPECT_TRUE(g.sendCycle());                // the cycle keeps counting
  EXPECT_FALSE(g.sendCycle());
}

TEST(ArmGate, AStandingArmRepeatsItsLastCommand)
{
  ArmGate g(1, at(5.0), fr5Steps());
  for (int i = 0; i < 10; ++i) {
    const auto d = g.decide(at(5.0), false);   // the same command every cycle
    EXPECT_EQ(d.action, Segment::Send);
    g.sent(d.target);
  }
  EXPECT_EQ(g.lastCommand()[0], 5.0);
}
