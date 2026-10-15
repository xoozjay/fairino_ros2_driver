#include "fr_robot_driver/gripper_motion.hpp"

#include <initializer_list>

#include <gtest/gtest.h>

using fr_robot_driver::GripperMotion;
using Outcome = fr_robot_driver::GripperOutcome;
using Source = fr_robot_driver::GripperSource;

namespace
{
GripperMotion::Time at(int ms)
{
  return GripperMotion::Time{} + std::chrono::milliseconds(ms);
}

struct Row
{
  int ms;
  bool active;
  int done;
  int position;
  int reported;  // percent of the stroke
  Source source = Source::Frame;
  std::optional<Outcome> outcome = std::nullopt;
};

// Rows are changes in the observed frame. Repeat the preceding frame each simulated millisecond
// between rows, checking that no completion slips through between the explicit expectations.
void replay(GripperMotion & motion, std::initializer_list<Row> rows)
{
  const Row * previous = nullptr;
  for (const auto & row : rows) {
    if (previous) {
      for (int ms = previous->ms + 1; ms < row.ms; ++ms) {
        const auto state = motion.frame(previous->active, previous->done, previous->position, at(ms));
        ASSERT_FALSE(state.outcome) << "unexpected completion at " << ms << " ms";
        ASSERT_NEAR(state.opening, previous->reported * 0.0004, 1e-12) << ms;
        ASSERT_EQ(state.source, previous->source) << ms;
      }
    }
    const auto state = motion.frame(row.active, row.done, row.position, at(row.ms));
    ASSERT_EQ(state.outcome, row.outcome) << row.ms;
    ASSERT_NEAR(state.opening, row.reported * 0.0004, 1e-12) << row.ms;
    ASSERT_EQ(state.source, row.source) << row.ms;
    previous = &row;
  }
}
}  // namespace

TEST(GripperMotion, NormalCloseAndOpen)
{
  // R1: 2026-10-14 X4, command-relative times rounded to ms.
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 100, 100}});
  motion.sent(50, at(0));
  replay(motion, {
    {0, true, 1, 100, 100},  // before the controller takes the call
    {15, true, 0, 0, 100, Source::HeldDuringMotion},
    {820, true, 0, 95, 95}, {1230, true, 0, 73, 73}, {1630, true, 0, 52, 52},
    {2030, true, 0, 50, 50}, {2230, true, 1, 50, 50},
    {2530, true, 1, 50, 50, Source::Frame, Outcome::Reached},
    {4999, true, 1, 50, 50},
  });
  motion.sent(100, at(5000));
  replay(motion, {
    {5015, true, 0, 0, 50, Source::HeldDuringMotion},
    {5870, true, 0, 54, 54}, {6280, true, 0, 76, 76}, {6680, true, 0, 97, 97},
    {7080, true, 0, 100, 100}, {7290, true, 1, 100, 100},
    {7590, true, 1, 100, 100, Source::Frame, Outcome::Reached},
  });
}

TEST(GripperMotion, InPlaceMoveKeepsItsOpeningThroughTheReset)
{
  // R2: 2026-10-14 E5, 50 -> 50 %. The old opening before the call is taken must not clear the guard.
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 50, 50}});
  motion.sent(50, at(0));
  replay(motion, {
    {0, true, 1, 50, 50},
    {15, true, 0, 0, 50, Source::HeldDuringMotion},
    {679, true, 1, 0, 50, Source::HeldDuringMotion},
    {889, true, 1, 50, 50},
    {1189, true, 1, 50, 50, Source::Frame, Outcome::Reached},
  });
}

TEST(GripperMotion, AGraspEndsOnceAndLaterFlagsDoNotEndAnotherGoal)
{
  // R3: 2026-10-14 X2. The action ends at 2; a subsequent 2 -> 0 -> 3 is raw telemetry only.
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 100, 100}});
  motion.sent(0, at(0));
  replay(motion, {
    {0, true, 0, 0, 100, Source::HeldDuringMotion},
    {2319, true, 2, 27, 27},
    {2619, true, 2, 27, 27, Source::Frame, Outcome::Grasped},
    {8589, true, 2, 23, 23}, {8789, true, 0, 23, 23}, {8989, true, 0, 6, 6},
    {9199, true, 3, 6, 6}, {9399, true, 3, 0, 0}, {10000, true, 3, 0, 0},
  });
}

TEST(GripperMotion, OpensFromAGrasp)
{
  // R3: 2026-10-14 X2, a separate opening from done=2.
  GripperMotion motion;
  replay(motion, {{-1, true, 2, 28, 28}});
  motion.sent(100, at(0));
  replay(motion, {
    {15, true, 0, 0, 28, Source::HeldDuringMotion},
    {908, true, 0, 32, 32}, {1308, true, 0, 54, 54}, {1718, true, 0, 78, 78},
    {2118, true, 0, 97, 97}, {2528, true, 0, 100, 100}, {2728, true, 1, 100, 100},
    {3028, true, 1, 100, 100, Source::Frame, Outcome::Reached},
  });
}

TEST(GripperMotion, ReplaysTrialAWithoutInferringTheCommandTimeoutFromIt)
{
  // R4: 2026-10-15 A. Only the observed frame ordering is evidence; the pull timing was rejected
  // by the maintainer. This tests no timeout theory and, as in X2, no second result after a grasp.
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 100, 100}});
  motion.sent(10, at(0));
  replay(motion, {
    {15, true, 0, 0, 100, Source::HeldDuringMotion},
    {903, true, 0, 95, 95}, {1313, true, 0, 73, 73}, {1713, true, 0, 49, 49},
    {2113, true, 0, 28, 28}, {2323, true, 2, 28, 28},
    {2623, true, 2, 28, 28, Source::Frame, Outcome::Grasped},
    {15868, true, 2, 30, 30}, {16077, true, 0, 30, 30}, {16268, true, 0, 19, 19},
    {16477, true, 3, 19, 19}, {16678, true, 3, 10, 10}, {17000, true, 3, 10, 10},
  });
}

TEST(GripperMotion, OldFlagsWhileOpeningAreNotCompletion)
{
  // R5: 2026-10-15, opening from 3 (G17), times relative to the command.
  GripperMotion motion;
  replay(motion, {{-1, true, 3, 10, 10}});
  motion.sent(100, at(0));
  replay(motion, {
    {18, true, 0, 0, 10, Source::HeldDuringMotion},
    {719, true, 3, 0, 10, Source::HeldDuringMotion}, {919, true, 3, 14, 14},
    {1119, true, 0, 14, 14}, {1319, true, 0, 36, 36}, {1729, true, 0, 60, 60},
    {2129, true, 0, 83, 83}, {2539, true, 0, 99, 99}, {2740, true, 1, 99, 99},
    {2939, true, 1, 100, 100},
    {3239, true, 1, 100, 100, Source::Frame, Outcome::Reached},
  });
}

TEST(GripperMotion, NoObjectMonitoringAfterTheCommandWindow)
{
  // R6: 2026-10-15 B'', max_time=3 s. Removing the object much later leaves the same frame.
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 100, 100}});
  motion.sent(10, at(0));
  replay(motion, {
    {11, true, 0, 0, 100, Source::HeldDuringMotion},
    {852, true, 0, 95, 95}, {1262, true, 0, 73, 73}, {1662, true, 0, 49, 49},
    {2072, true, 0, 28, 28}, {2272, true, 2, 28, 28},
    {2572, true, 2, 28, 28, Source::Frame, Outcome::Grasped},
    {3000, true, 2, 28, 28}, {100000, true, 2, 28, 28},
  });
}

TEST(GripperMotion, ActivationInfersOpenUntilTheFirstMove)
{
  // R7: 2026-10-14 X4 activation, active at 739 ms but still reporting 0 %, done=0.
  GripperMotion motion;
  replay(motion, {
    {22, false, 0, 0, 100, Source::UnknownNotActive},
    {739, true, 0, 0, 100, Source::InferredOpenAfterActivation},
    {12000, true, 0, 0, 100, Source::InferredOpenAfterActivation},
  });
  motion.sent(50, at(12001));
  replay(motion, {
    {12016, true, 0, 0, 100, Source::InferredOpenAfterActivation},
    {12821, true, 0, 95, 95},
  });
}

TEST(GripperMotion, ReactivationReplacesThePreviousKnownOpening)
{
  // Contract: activating opens fully, so an opening known before losing activation is obsolete.
  GripperMotion motion;
  replay(motion, {
    {0, true, 1, 50, 50}, {1, false, 0, 0, 100, Source::UnknownNotActive},
    {2, true, 0, 0, 100, Source::InferredOpenAfterActivation},
  });
}

TEST(GripperMotion, CompletionRequiresStartFlagsAndAPlausibleOpening)
{
  // Contract boundaries, not an additional firmware simulation or a guard for target 0 %.
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 50, 50}});
  motion.sent(100, at(0));
  replay(motion, {
    {0, true, 1, 50, 50}, {301, true, 1, 50, 50},  // stable old flags, no start
    {400, true, 0, 0, 50, Source::HeldDuringMotion},
    {701, true, 0, 0, 50, Source::HeldDuringMotion},  // stable zero is not completion
    {800, true, 1, 0, 50, Source::HeldDuringMotion},
    {1101, true, 1, 0, 50, Source::HeldDuringMotion},  // target > 0: reset position is not plausible
    {1200, true, 1, 100, 100},
    {1500, true, 1, 100, 100, Source::Frame, Outcome::Reached},
  });
}

TEST(GripperMotion, EndFlagsMapToOneResult)
{
  // Contract: target 0 remains valid; 1=reached, 2=stuck, 3=stuck then reached, others=unknown.
  for (int flags : {1, 2, 3, 4}) {
    GripperMotion motion;
    replay(motion, {{-1, true, 1, 100, 100}});
    motion.sent(0, at(0));
    const auto expected = flags == 1 ? Outcome::Reached : flags == 2 ? Outcome::Grasped :
      flags == 3 ? Outcome::Dropped : Outcome::Other;
    replay(motion, {
      {0, true, 0, 0, 100, Source::HeldDuringMotion}, {10, true, 0, 20, 20},
      {20, true, flags, 0, 0}, {320, true, flags, 0, 0, Source::Frame, expected},
      {1000, true, flags, 0, 0},
    });
  }
}

TEST(GripperMotion, DeadlinesUseSuppliedTime)
{
  // Contract: deadlines are exercised without waiting 1.5 or 30 real seconds.
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 50, 50}});
  motion.sent(50, at(0));
  replay(motion, {
    {0, true, 1, 50, 50}, {1500, true, 1, 50, 50},
    {1501, true, 1, 50, 50, Source::Frame, Outcome::NeverStarted},
  });
  motion.sent(0, at(2000));
  replay(motion, {
    {2000, true, 0, 0, 50, Source::HeldDuringMotion},
    {32000, true, 0, 0, 50, Source::HeldDuringMotion},
    {32001, true, 0, 0, 50, Source::HeldDuringMotion, Outcome::TimedOut},
  });
}

TEST(GripperMotion, ResetForgetsTheMotionButKeepsTheOpening)
{
  GripperMotion motion;
  replay(motion, {{-1, true, 1, 50, 50}});
  motion.sent(100, at(0));
  replay(motion, {{0, true, 0, 0, 50, Source::HeldDuringMotion}});
  motion.resetMotion();
  replay(motion, {
    {1, true, 1, 0, 0}, {301, true, 1, 0, 0}, {400, true, 1, 50, 50},
  });
  motion.sent(50, at(401));
  replay(motion, {
    {402, true, 0, 0, 50, Source::HeldDuringMotion}, {450, true, 1, 50, 50},
    {750, true, 1, 50, 50, Source::Frame, Outcome::Reached},
  });
}
