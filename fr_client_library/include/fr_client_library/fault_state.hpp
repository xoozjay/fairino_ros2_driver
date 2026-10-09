// Fault classification of a state frame (docs/fault_handling.md, section 3).
//
// Pure protocol knowledge, no ROS and no I/O: the hardware plugin feeds it validated frames.
// The error codes (main_code / sub_code) are exported as they are; nothing here translates them.
#pragma once

#include <string>

#include "fr_client_library/robot_state_384.hpp"

namespace fr_client_library
{
// Ordered by severity: the worse one wins (COMM > ESTOP > ROBOT_FAULT), because with an
// untrustworthy link nothing in the frames can be trusted either.
enum class FaultClass { None = 0, RobotFault = 1, Estop = 2, Comm = 3 };

constexpr const char * toString(FaultClass f)
{
  switch (f) {
    case FaultClass::None: return "none";
    case FaultClass::RobotFault: return "robot fault";
    case FaultClass::Estop: return "emergency stop";
    case FaultClass::Comm: return "communication fault";
  }
  return "?";
}

// Classifies a valid, fresh frame. Never returns Comm: the link is judged from frame freshness.
// rbtEnableState != 1 and robot_state == 4 (drag teaching) count as faults: no servo motion then.
// ❓ the encoding of rbtEnableState is only known from the simulator (1 = enabled).
inline FaultClass classifyState(const RobotState384 & s)
{
  if (s.EmergencyStop != 0) {
    return FaultClass::Estop;
  }
  if (s.main_code != 0 || s.collisionState != 0 || s.safety_stop0_state != 0 ||
    s.safety_stop1_state != 0 || s.rbtEnableState != 1 || s.robot_state == 4)
  {
    return FaultClass::RobotFault;
  }
  return FaultClass::None;
}

// Which field tripped, for log messages ("main_code=1 sub_code=23").
inline std::string describeFault(const RobotState384 & s)
{
  std::string out;
  const auto add = [&out](const std::string & what) {
      out += (out.empty() ? "" : ", ") + what;
    };
  if (s.EmergencyStop != 0) {add("emergency stop");}
  if (s.main_code != 0) {
    add("main_code=" + std::to_string(s.main_code) + " sub_code=" + std::to_string(s.sub_code));
  }
  if (s.collisionState != 0) {add("collision");}
  if (s.safety_stop0_state != 0) {add("safety stop SI0");}
  if (s.safety_stop1_state != 0) {add("safety stop SI1");}
  if (s.rbtEnableState != 1) {add("not enabled (rbtEnableState=" + std::to_string(s.rbtEnableState) + ")");}
  if (s.robot_state == 4) {add("drag teaching");}
  return out.empty() ? "no fault" : out;
}

// The state frame's motion queue length (mc_queue_len): how many ServoJ segments the firmware
// still has queued (docs/sdk_quirks.md S15, S16).
inline int servoQueueLength(const RobotState384 & s) {return s.mc_queue_len;}

}  // namespace fr_client_library
