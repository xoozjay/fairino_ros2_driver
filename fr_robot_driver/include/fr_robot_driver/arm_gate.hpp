// What the plugin's write() does with the arm's commands: this class decides, the plugin does it.
//
// The rules, and why they are (docs/sdk_quirks.md, docs/fault_handling.md section 6):
//  * ServoJ moves the arm directly, without the global speed and acceleration limits (S14). The only
//    protection on the ROS side is the URDF's velocity limits plus a per-cycle step check: a single
//    command may not jump further than the URDF's fastest joint can travel in one cmdT (S11: a jump
//    that does makes the controller fault, and then every ServoJ is refused).
//  * The controller manager may run faster than cmdT: only every ratio_-th cycle is a segment.
//  * While the gripper moves the controller holds ServoJ back (G9), so the arm stays where it is:
//    commands that would move it are not sent. The trajectory controller pauses meanwhile
//    (speed_scaling_factor 0, read()); a controller that ignores the scaling runs ahead and the
//    step check stops it.
//
// No ROS, no I/O, no clock: the plugin feeds it the commands it read and the answers it knows, and
// turns the decision into ros2_control calls and log messages. Tested without sockets or wall time.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

#include "fr_client_library/robot_api.hpp"  // JointArray

namespace fr_robot_driver
{
using fr_client_library::JointArray;
enum class Segment { Hold, Send, Refuse };  // repeat the last command, enqueue a new one, refuse it

// Why a command was refused: which joint, from where to where, and what the limit was.
struct StepViolation
{
  int joint = 0;
  double from_deg = 0;
  double to_deg = 0;
  double limit_deg = 0;
};

// One send cycle's decision: `target` is the segment to enqueue when action == Send, and `violation`
// describes the refusal when action == Refuse.
struct SegmentDecision
{
  Segment action = Segment::Hold;
  JointArray target{};
  StepViolation violation{};
};

class ArmGate
{
public:
  // ratio: controller cycles per ServoJ segment (1 = one segment per cycle).
  explicit ArmGate(int ratio, const fr_client_library::JointArray & initial_deg,
    const std::array<double, 6> & max_step_deg)
  : ratio_(std::max(1, ratio)), max_step_deg_(max_step_deg)
  {
    last_cmd_deg_ = initial_deg;
  }

  // Does the command differ from the segment last sent? While it does, the controller is still
  // asking for a position the arm has not reached yet, so the gripper does not start (G8, G9: the
  // controller holds back ServoJ while a gripper motion runs, and the arm goes first).
  bool commandedMoved(const fr_client_library::JointArray & command_deg) const
  {
    return command_deg != last_cmd_deg_;
  }

  // The command for this cycle: the parked one while the gripper moves, the commanded one otherwise.
  fr_client_library::JointArray target(
    const fr_client_library::JointArray & command_deg, bool gripper_busy) const
  {
    return gripper_busy ? last_cmd_deg_ : command_deg;
  }

  // Decides the segment for a send cycle (call only when sendCycle()).
  SegmentDecision decide(const fr_client_library::JointArray & command_deg, bool gripper_busy) const
  {
    const JointArray target_deg = target(command_deg, gripper_busy);
    SegmentDecision d{Segment::Send, target_deg, {}};
    if (check(target_deg, d.violation)) {
      d.action = Segment::Refuse;
    }
    return d;
  }

  // Is this cycle a segment? (ratio controller cycles per cmdT.)
  bool sendCycle()
  {
    return write_count_++ % static_cast<unsigned long>(ratio_) == 0;
  }

  // A segment was enqueued: remember it as the last one sent.
  void sent(const fr_client_library::JointArray & target_deg) {last_cmd_deg_ = target_deg;}

  const JointArray & lastCommand() const {return last_cmd_deg_;}

  // The first joint whose step exceeds its limit; the returned bool says whether there is one.
  bool check(const fr_client_library::JointArray & target_deg, StepViolation & out) const
  {
    for (std::size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(target_deg[i]) ||
        std::abs(target_deg[i] - last_cmd_deg_[i]) > max_step_deg_[i])
      {
        out = {static_cast<int>(i), last_cmd_deg_[i], target_deg[i], max_step_deg_[i]};
        return true;
      }
    }
    return false;
  }

private:
  int ratio_;
  std::array<double, 6> max_step_deg_;
  JointArray last_cmd_deg_{};
  unsigned long write_count_ = 0;
};

}  // namespace fr_robot_driver
