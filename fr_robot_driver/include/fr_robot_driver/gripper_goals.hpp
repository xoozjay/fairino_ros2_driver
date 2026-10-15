// The plugin's gripper action (control_msgs/ParallelGripperCommand): which goals are accepted, what
// may be sent, and how a motion ends. The plugin owns the goal handles and the network; this class
// decides (sdk_quirks G1-G17, fr_robot_driver/README.md "夹爪").
//
// Why the action lives in the driver at all: the upstream parallel_gripper_action_controller only
// watches the opening's velocity, so it cannot tell "an object is grasped" from "waiting for the arm
// to stand still", and it leaves the last target in the command interface after an abort, which made
// the gripper move on a stale goal (G13-G15, E4 of the retest plan). The completion is therefore read
// from the state frame by GripperMotion, and this class maps it onto the action's result.
//
// No ROS, no I/O, no clock: the plugin hands it goals, epochs and outcomes; it answers with what to
// do. Tested without an action server, sockets or wall time.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "fr_client_library/gripper_sender.hpp"  // GripperCommand
#include "fr_robot_driver/gripper_motion.hpp"    // GripperOutcome

namespace fr_robot_driver
{
using fr_client_library::GripperCommand;
// A goal the action server accepted, in the plugin's slot (one at a time).
struct GripperGoal
{
  uint64_t id = 0;                     // 0: none
  uint64_t epoch = 0;                  // gripper_epoch_ when it was accepted
  GripperCommand command;
  bool sent = false;                   // its MoveGripper went out
};

// How a goal ended, and what the result should say.
enum class GoalEnd { Running, Succeeded, Aborted, Canceled };

struct GoalEnding
{
  GoalEnd end = GoalEnd::Running;
  std::string why;                     // for the log message
  bool reached = false;                // result.reached_goal
  bool stalled = false;                // result.stalled (an object is grasped)
};

class GripperGoals
{
public:
  // Opening [m] of one PGI-140 jaw: 0 closed, 0.04 fully open (the URDF's joint range).
  static constexpr double kStroke = GripperMotion::kStroke;
  static constexpr int kProtocolVelocity = 50;      // fixed: the speed parameter has no effect (G12)
  static constexpr int kDefaultForcePercent = 20;   // 28 N of the 140 N full scale: glassware
  static constexpr double kForcePerPercent = 1.4;   // N per percent of MoveGripper's force

  // A goal's command: the opening clamped to the stroke, the fixed protocol speed, the force from
  // the goal's effort (N -> percent, [1, 100]; none or invalid -> the default).
  static GripperCommand command(double opening_m, double effort_n)
  {
    GripperCommand c;
    c.position = static_cast<int>(
      std::lround(std::clamp(opening_m, 0.0, kStroke) / kStroke * 100.0));
    c.velocity = kProtocolVelocity;
    const double n = std::isfinite(effort_n) ? effort_n : 0.0;
    c.force = n > 0.0 ? static_cast<int>(std::clamp<long>(std::lround(n / kForcePerPercent), 1, 100)) :
      kDefaultForcePercent;
    return c;
  }

  // Why a new goal must be refused; empty if it is acceptable. (The hardware must be active, the
  // gripper usable; the goal must ask for exactly one finite opening.)
  static std::string refuse(
    const std::array<double, 1> & position, bool hardware_active, bool gripper_usable)
  {
    if (!hardware_active || !gripper_usable) {
      return "the hardware is not active, or the gripper is not active or reports a fault";
    }
    if (!std::isfinite(position[0])) {
      return "needs exactly one finite position";
    }
    return {};
  }

  // ---- the slot: the plugin holds its mutex while calling these ----

  // Accepts a goal into the slot. Returns the id of the goal it replaces (0: none), so the caller
  // can end that one as preempted: the controller cannot interrupt a motion (G15), so the new goal
  // waits for the one in progress.
  uint64_t accept(const GripperCommand & c, uint64_t epoch, uint64_t & next_id)
  {
    const uint64_t replaced = goal_.id;
    goal_ = GripperGoal{++next_id, epoch, c, false};
    return replaced;
  }

  const GripperGoal & goal() const {return goal_;}
  bool hasGoal() const {return goal_.id != 0;}
  void forget() {goal_ = GripperGoal{};}

  // May this goal's MoveGripper go out? The arm goes first (G8, G9): a goal waits while the arm is
  // still being commanded, while the gripper is busy, and until the servo stream is paused and the
  // firmware queue is empty (the controller holds MoveGripper back while ServoJ flows, G8).
  bool maySend(uint64_t epoch, bool gripper_usable, bool gripper_busy, int idle_frames,
    bool arm_moves) const
  {
    return goal_.id != 0 && !goal_.sent && goal_.epoch == epoch && gripper_usable && !gripper_busy &&
      idle_frames >= 2 && !arm_moves;
  }

  // The goal's MoveGripper went out.
  void markSent() {goal_.sent = true;}

  // ---- the executor side: what to do with the goal in the slot now ----

  // The goal was canceled (a motion that went out runs to its end, G15).
  GoalEnding canceled() const
  {
    return {GoalEnd::Canceled, goal_.sent ?
      "canceled; the motion already sent runs to its end" : "canceled before it was sent", false, false};
  }

  // The epoch moved: the hardware was (de)activated or failed, or the gripper became unusable or was
  // activated. The goal's value is not kept: it never runs, not even once things are back.
  GoalEnding stale(uint64_t epoch) const
  {
    if (goal_.epoch == epoch) {
      return {GoalEnd::Running, {}, false, false};
    }
    return {GoalEnd::Aborted, "the hardware was (de)activated or failed, or the gripper became "
      "unusable or was activated", false, false};
  }

  // The sender thread is gone: a goal that waits would wait forever.
  GoalEnding senderGone() const
  {
    return {GoalEnd::Aborted, "the gripper sender thread is gone", false, false};
  }

  // `finished` is (goal id << 3) | GripperOutcome, set when the frame showed that motion ending.
  // Running when it is another goal's outcome.
  GoalEnding outcome(uint64_t finished) const
  {
    if (!goal_.sent || (finished >> 3) != goal_.id) {
      return {GoalEnd::Running, {}, false, false};
    }
    switch (static_cast<GripperOutcome>(finished & 7u)) {
      case GripperOutcome::Reached:
        return {GoalEnd::Succeeded, "reached", true, false};
      case GripperOutcome::Grasped:
        return {GoalEnd::Succeeded, "an object is grasped", false, true};
      case GripperOutcome::Dropped:
        return {GoalEnd::Aborted, "the object was dropped", false, false};
      case GripperOutcome::Other:
        return {GoalEnd::Aborted, "unknown motion_done value", false, false};
      case GripperOutcome::NeverStarted:
        return {GoalEnd::Aborted, "the frame never showed the motion start within 1.5 s", false, false};
      case GripperOutcome::TimedOut:
        return {GoalEnd::Aborted, "not done within 30 s", false, false};
    }
    return {GoalEnd::Aborted, "unknown outcome", false, false};
  }

private:
  GripperGoal goal_;
};

}  // namespace fr_robot_driver
