// Gripper state inferred from the controller's frames (sdk_quirks G5, G10, G13, G14, G17).
// No I/O or clock reads: the caller supplies time, so recorded frames can be replayed directly.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>

namespace fr_robot_driver
{
// A reported opening may be held from a previous frame or inferred after activation.
enum class GripperSource { Frame, HeldDuringMotion, InferredOpenAfterActivation, UnknownNotActive };

// Values also fit the plugin's (goal id << 3) | outcome handover to its action timer.
enum class GripperOutcome : uint64_t { Reached = 1, Grasped, Dropped, Other, NeverStarted, TimedOut };

class GripperMotion
{
public:
  using Time = std::chrono::steady_clock::time_point;
  static constexpr double kStroke = 0.04;  // one PGI-140 jaw, metres

  struct State
  {
    double opening;
    GripperSource source;
    std::optional<GripperOutcome> outcome;  // only on the frame that ends our motion
  };

  void sent(int target_percent, Time t)
  {
    moving_ = true;
    posted_at_ = t;
    target_ = target_percent;
    seen_running_ = false;
    awaiting_nonzero_ = target_percent != 0;
    frame_state_ = -1;
  }

  // Hardware activation starts a new command epoch, but keeps the last known opening.
  void resetMotion()
  {
    moving_ = false;
    awaiting_nonzero_ = false;
  }

  State frame(bool active, int done, int position, Time t)
  {
    if (active && !was_active_) {
      known_from_activation_ = known_ >= 0.0;
      awaiting_nonzero_ = false;
    }
    was_active_ = active;

    std::optional<GripperOutcome> outcome;
    if (moving_) {
      seen_running_ = seen_running_ || done == 0;
      const int frame_state = done * 1000 + position;
      if (frame_state != frame_state_) {
        frame_state_ = frame_state;
        unchanged_since_ = t;
      }
      // Old flags briefly reappear after a command. Require flags AND opening to settle (G17).
      const bool settled = t - unchanged_since_ >= std::chrono::milliseconds(300);
      const bool plausible = target_ == 0 || position != 0;
      if (seen_running_ && done != 0 && settled && plausible) {
        outcome = done == 1 ? GripperOutcome::Reached : done == 2 ? GripperOutcome::Grasped :
          done == 3 ? GripperOutcome::Dropped : GripperOutcome::Other;
      } else if (!seen_running_ && t - posted_at_ > std::chrono::milliseconds(1500)) {
        outcome = GripperOutcome::NeverStarted;
      } else if (t - posted_at_ > std::chrono::seconds(30)) {
        outcome = GripperOutcome::TimedOut;
      }
      if (outcome) {moving_ = false;}
    }
    // A pre-command frame still shows the old opening; it cannot clear the zero-position guard.
    if (position != 0 && (!moving_ || seen_running_)) {
      awaiting_nonzero_ = false;
    }

    State state{0.0, GripperSource::Frame, outcome};
    if (!active) {
      // Unknown: report the widest outline for collision checking (G1, G11).
      state.opening = kStroke;
      state.source = GripperSource::UnknownNotActive;
    } else if (position == 0 && (done == 0 || awaiting_nonzero_)) {
      if (known_ >= 0.0 && !known_from_activation_) {
        state.opening = known_;
        state.source = GripperSource::HeldDuringMotion;
      } else {
        // Activation always opens fully; its frame can still say 0 % (G5).
        state.opening = kStroke;
        state.source = GripperSource::InferredOpenAfterActivation;
      }
    } else {
      state.opening = std::min(position, 100) / 100.0 * kStroke;
      known_ = state.opening;
      known_from_activation_ = false;
    }
    return state;
  }

private:
  bool moving_ = false;
  Time posted_at_{};
  int target_ = 0;
  bool seen_running_ = false;
  int frame_state_ = -1;
  Time unchanged_since_{};
  double known_ = -1.0;
  bool known_from_activation_ = false;
  bool awaiting_nonzero_ = false;
  bool was_active_ = false;
};
}  // namespace fr_robot_driver
