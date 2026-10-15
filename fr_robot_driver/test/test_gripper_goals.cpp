// GripperGoals: the action's rules without an action server, sockets or wall time (sdk_quirks G1-G17).
#include <gtest/gtest.h>

#include <cmath>

#include "fr_robot_driver/gripper_goals.hpp"

using namespace fr_robot_driver;
using fr_client_library::GripperCommand;

namespace
{
constexpr uint64_t kEpoch = 1;

GripperCommand cmd(int percent) {return GripperGoals::command(percent * 0.0004, 0.0);}
}  // namespace

TEST(GripperGoals, TheCommandConvertsOpeningAndForce)
{
  const auto open_half = GripperGoals::command(0.02, 0.0);
  EXPECT_EQ(open_half.position, 50);
  EXPECT_EQ(open_half.velocity, GripperGoals::kProtocolVelocity);   // fixed: the speed is ignored
  EXPECT_EQ(open_half.force, GripperGoals::kDefaultForcePercent);

  EXPECT_EQ(GripperGoals::command(0.0, 0.0).position, 0);
  EXPECT_EQ(GripperGoals::command(0.04, 0.0).position, 100);
  EXPECT_EQ(GripperGoals::command(0.5, 0.0).position, 100);          // clamped to the stroke
  EXPECT_EQ(GripperGoals::command(0.02, 28.0).force, 20);            // N / 1.4
  EXPECT_EQ(GripperGoals::command(0.02, 0.0).force, 20);             // none given: the default
  EXPECT_EQ(GripperGoals::command(0.02, 0.001).force, 1);            // clamped to 1 %
  EXPECT_EQ(GripperGoals::command(0.02, 5000.0).force, 100);         // clamped to 100 %
  EXPECT_EQ(GripperGoals::command(0.02, std::nan("")).force, GripperGoals::kDefaultForcePercent);
}

TEST(GripperGoals, AGoalIsRefusedUnlessTheHardwareAndGripperAreUsable)
{
  const std::array<double, 1> ok{0.02};
  EXPECT_TRUE(GripperGoals::refuse(ok, true, true).empty());
  EXPECT_FALSE(GripperGoals::refuse(ok, false, true).empty());       // hardware not active
  EXPECT_FALSE(GripperGoals::refuse(ok, true, false).empty());       // gripper inactive or faulted
  const std::array<double, 1> nan{std::nan("")};
  EXPECT_FALSE(GripperGoals::refuse(nan, true, true).empty());
}

TEST(GripperGoals, ANewGoalPreemptsTheWaitingOneAndTheSentOneRunsOn)
{
  GripperGoals g;
  uint64_t ids = 0;
  EXPECT_EQ(g.accept(cmd(100), kEpoch, ids), 0u);                    // nothing to replace
  EXPECT_EQ(g.goal().id, 1u);
  EXPECT_EQ(g.accept(cmd(50), kEpoch, ids), 1u);                     // the waiting one is replaced
  EXPECT_EQ(g.goal().id, 2u);
  EXPECT_TRUE(g.maySend(kEpoch, true, false, 2, false));
  g.markSent();
  EXPECT_FALSE(g.maySend(kEpoch, true, false, 2, false));            // already sent
  EXPECT_EQ(g.accept(cmd(0), kEpoch, ids), 2u);                      // and the sent one too: the motion
  EXPECT_EQ(g.goal().id, 3u);                                        // in progress runs to its end (G15)
}

TEST(GripperGoals, AGoalWaitsForTheArmAndForTheStreamToBeIdle)
{
  GripperGoals g;
  uint64_t ids = 0;
  g.accept(cmd(0), kEpoch, ids);
  EXPECT_TRUE(g.maySend(kEpoch, true, false, 2, false));             // stream idle, arm at rest
  EXPECT_FALSE(g.maySend(kEpoch, true, false, 2, true));             // the arm is being commanded
  EXPECT_FALSE(g.maySend(kEpoch, true, false, 1, false));            // not yet idle for two frames
  EXPECT_FALSE(g.maySend(kEpoch, true, false, 0, false));
  EXPECT_FALSE(g.maySend(kEpoch, true, true, 5, false));             // the gripper is busy
  EXPECT_FALSE(g.maySend(kEpoch, false, false, 5, false));           // the gripper is unusable
  EXPECT_FALSE(g.maySend(kEpoch + 1, true, false, 5, false));        // stale epoch
}

TEST(GripperGoals, TheEndingFollowsTheFramesFlags)
{
  GripperGoals g;
  uint64_t ids = 0;
  g.accept(cmd(0), kEpoch, ids);
  g.markSent();
  const uint64_t id = g.goal().id;
  const auto with = [&](GripperOutcome o) {return g.outcome((id << 3) | static_cast<uint64_t>(o));};

  EXPECT_EQ(with(GripperOutcome::Reached).end, GoalEnd::Succeeded);
  EXPECT_TRUE(with(GripperOutcome::Reached).reached);
  EXPECT_FALSE(with(GripperOutcome::Reached).stalled);

  const auto grasp = with(GripperOutcome::Grasped);
  EXPECT_EQ(grasp.end, GoalEnd::Succeeded);                          // an object is grasped
  EXPECT_TRUE(grasp.stalled);
  EXPECT_FALSE(grasp.reached);

  EXPECT_EQ(with(GripperOutcome::Dropped).end, GoalEnd::Aborted);     // lost it on the way
  EXPECT_EQ(with(GripperOutcome::Other).end, GoalEnd::Aborted);       // unknown motion_done value
  EXPECT_EQ(with(GripperOutcome::NeverStarted).end, GoalEnd::Aborted);  // no motion within 1.5 s
  EXPECT_EQ(with(GripperOutcome::TimedOut).end, GoalEnd::Aborted);      // not done within 30 s

  EXPECT_EQ(g.outcome(0).end, GoalEnd::Running);                      // no motion ended yet
  EXPECT_EQ(g.outcome(((id + 1) << 3) | 1u).end, GoalEnd::Running);    // another goal's outcome
}

TEST(GripperGoals, ACanceledGoalKnowsWhetherItsMotionWentOut)
{
  GripperGoals g;
  uint64_t ids = 0;
  g.accept(cmd(50), kEpoch, ids);
  const auto before = g.canceled();
  EXPECT_EQ(before.end, GoalEnd::Canceled);
  EXPECT_NE(before.why.find("before it was sent"), std::string::npos);
  g.markSent();
  const auto after = g.canceled();                                   // the motion runs to its end (G15)
  EXPECT_EQ(after.end, GoalEnd::Canceled);
  EXPECT_NE(after.why.find("runs to its end"), std::string::npos);
}

TEST(GripperGoals, AStaleEpochEndsTheGoalAndASentGoneSenderEndsItToo)
{
  GripperGoals g;
  uint64_t ids = 0;
  g.accept(cmd(50), kEpoch, ids);
  EXPECT_EQ(g.stale(kEpoch).end, GoalEnd::Running);
  const auto stale = g.stale(kEpoch + 1);   // activated, or the hardware was (de)activated or failed
  EXPECT_EQ(stale.end, GoalEnd::Aborted);
  EXPECT_EQ(g.senderGone().end, GoalEnd::Aborted);                    // would wait forever otherwise
}

TEST(GripperGoals, ForgettingLeavesAnEmptySlot)
{
  GripperGoals g;
  uint64_t ids = 0;
  g.accept(cmd(50), kEpoch, ids);
  EXPECT_TRUE(g.hasGoal());
  g.forget();
  EXPECT_FALSE(g.hasGoal());
  EXPECT_FALSE(g.maySend(kEpoch, true, false, 5, false));
  EXPECT_EQ(g.outcome((1u << 3) | 1u).end, GoalEnd::Running);
}
