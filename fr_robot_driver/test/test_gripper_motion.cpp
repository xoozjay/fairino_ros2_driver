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
    {1, true, 0, 0, 50, Source::HeldDuringMotion},
    {301, true, 0, 0, 50, Source::HeldDuringMotion}, {400, true, 1, 50, 50},
  });
  motion.sent(50, at(401));
  replay(motion, {
    {402, true, 0, 0, 50, Source::HeldDuringMotion}, {450, true, 1, 50, 50},
    {750, true, 1, 50, 50, Source::Frame, Outcome::Reached},
  });
}
