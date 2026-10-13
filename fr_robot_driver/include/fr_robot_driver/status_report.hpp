// What the driver tells the upper layer about the robot and itself (the `~/status` topic).
//
// Pure: turns what the plugin knows (lifecycle state, why it declared a communication fault, the
// newest state frame and its age) into a diagnostic_msgs/DiagnosticStatus. No ROS node, no I/O.
// The tracker remembers since when a condition holds and which fault came before it.
// The driver only reports; nothing here makes it act (docs/fault_handling.md section 7).
#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "fr_client_library/fault_state.hpp"

namespace fr_robot_driver
{
enum class HardwareState { Unconfigured, Inactive, Active, Finalized };

// Why read()/write() declared a communication fault (None: they did not).
enum class CommReason { None, NoFrame, LinkLost, ServoFailed, EnqueueFailed };

inline const char * toString(HardwareState s)
{
  switch (s) {
    case HardwareState::Unconfigured: return "unconfigured";
    case HardwareState::Inactive: return "inactive";
    case HardwareState::Active: return "active";
    case HardwareState::Finalized: return "finalized";
  }
  return "?";
}

inline const char * toString(CommReason r)
{
  switch (r) {
    case CommReason::None:
    case CommReason::NoFrame: return "no state frame in time";
    case CommReason::LinkLost: return "state connection lost";
    case CommReason::ServoFailed: return "servo stream connection failed";
    case CommReason::EnqueueFailed: return "servo stream queue failed";
  }
  return "?";
}

// The fault to act on and report: the frame's, or RobotFault for the driver's own protective stop
// (a command refused by the step check), which stays until the next activation.
struct Fault
{
  fr_client_library::FaultClass fault_class = fr_client_library::FaultClass::None;
  std::string detail;
};

inline Fault effectiveFault(const fr_client_library::RobotState384 & s, bool step_fault)
{
  const auto fc = fr_client_library::classifyState(s);
  if (fc == fr_client_library::FaultClass::None && step_fault) {
    return {fr_client_library::FaultClass::RobotFault, "a command was refused by the step check"};
  }
  return {fc, fr_client_library::describeFault(s)};
}

struct StatusInput
{
  HardwareState hardware = HardwareState::Unconfigured;
  CommReason comm = CommReason::None;
  bool step_fault = false;                 // our own step check refused a command
  bool link_up = false;                    // frames are arriving; `frame` is valid
  fr_client_library::RobotState384 frame{};
  long frame_age_ms = -1;                  // age of the newest frame ever received; -1: none
};

// "2026-10-12T08:15:30.123Z"
inline std::string isoTime(std::chrono::system_clock::time_point t)
{
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
  const std::time_t sec = static_cast<std::time_t>(ms.count() / 1000);
  std::tm tm{};
  gmtime_r(&sec, &tm);
  char date[32];
  std::strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &tm);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%s.%03dZ", date, static_cast<int>(ms.count() % 1000));
  return buf;
}

class StatusTracker
{
public:
  using Status = diagnostic_msgs::msg::DiagnosticStatus;

  // Rebuilds the status. True when it differs from the previous call's in anything but the frame
  // age (that is when it should be published at once).
  bool update(const StatusInput & in, std::chrono::system_clock::time_point now)
  {
    using fr_client_library::FaultClass;
    const bool servo_failed = in.hardware != HardwareState::Unconfigured &&
      (in.comm == CommReason::ServoFailed || in.comm == CommReason::EnqueueFailed);

    FaultClass fc = FaultClass::None;
    uint8_t level = Status::OK;
    std::string message = "ok";
    if (in.link_up && !servo_failed) {
      const Fault f = effectiveFault(in.frame, in.step_fault && in.hardware != HardwareState::Unconfigured);
      fc = f.fault_class;
      if (fc != FaultClass::None) {
        level = Status::ERROR;
        message = std::string(fr_client_library::toString(fc)) + ": " + f.detail;
      } else if (in.hardware != HardwareState::Active && in.hardware != HardwareState::Inactive) {
        level = Status::WARN;
        message = "controller reachable, hardware " + std::string(toString(in.hardware)) +
          ": configure and activate to resume";
      }
    } else if (in.hardware == HardwareState::Unconfigured && in.comm == CommReason::None) {
      level = Status::WARN;
      message = "not connected: hardware unconfigured";
    } else {
      // The driver only sees the link break; the controller itself may have lost power.
      fc = FaultClass::Comm;
      level = Status::ERROR;
      message = std::string(fr_client_library::toString(fc)) + ": " +
        toString(in.comm) +
        " (link to the controller interrupted; possible causes: controller power loss, e.g. an "
        "emergency stop, or a network failure)";
    }

    // the condition: since when does this hold; a fault that ends becomes `previous`
    const std::string condition = std::to_string(static_cast<int>(fc)) + message;
    if (!have_ || condition != condition_) {
      if (have_ && fault_class_ != FaultClass::None) {
        previous_ = message_ + " (from " + since_ + " to " + isoTime(now) + ")";
      }
      since_ = isoTime(now);
      condition_ = condition;
    }
    fault_class_ = fc;
    message_ = message;

    Status s;
    s.name = name_;
    s.hardware_id = hardware_id_;
    s.level = level;
    s.message = message;
    const auto add = [&s](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = k;
        kv.value = v;
        s.values.push_back(kv);
      };
    const auto frame = [&](const std::string & v) {return in.link_up ? v : std::string("unknown");};
    const auto & f = in.frame;
    add("fault_class", std::to_string(static_cast<int>(fc)));
    add("main_code", frame(std::to_string(f.main_code)));
    add("sub_code", frame(std::to_string(f.sub_code)));
    add("enabled", frame(f.rbtEnableState == 1 ? "1" : "0"));
    add("estop", frame(std::to_string(f.EmergencyStop)));
    add("safety_stop0", frame(std::to_string(f.safety_stop0_state)));
    add("safety_stop1", frame(std::to_string(f.safety_stop1_state)));
    add("hardware_state", toString(in.hardware));
    add("link", in.link_up ? "up" : "down");
    const std::size_t stable = s.values.size();   // everything above decides "changed"
    add("servo_queue_len", frame(std::to_string(fr_client_library::servoQueueLength(f))));
    add("last_frame_age_ms", in.frame_age_ms < 0 ? "none" : std::to_string(in.frame_age_ms));
    add("since", since_);
    add("previous", previous_);

    std::string key = std::to_string(level) + message;
    for (std::size_t i = 0; i < stable; ++i) {key += "|" + s.values[i].value;}
    const bool changed = !have_ || key != key_;
    key_ = key;
    have_ = true;
    status_ = s;
    return changed;
  }

  const Status & status() const {return status_;}

  void identify(const std::string & name, const std::string & hardware_id)
  {
    name_ = name;
    hardware_id_ = hardware_id;
  }

private:
  bool have_ = false;
  std::string name_, hardware_id_;
  std::string condition_, key_, message_, since_;
  std::string previous_ = "none";
  fr_client_library::FaultClass fault_class_ = fr_client_library::FaultClass::None;
  Status status_;
};

}  // namespace fr_robot_driver
