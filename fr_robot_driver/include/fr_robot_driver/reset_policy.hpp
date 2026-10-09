// When may the operator's reset_error request be forwarded to the controller?
// docs/fault_handling.md section 5: F3 (never reset automatically), F4 (an emergency stop can only
// be cleared by restarting the controller), and no reset on an untrustworthy link.
#pragma once

#include "fr_client_library/fault_state.hpp"

namespace fr_robot_driver
{
enum class ResetDecision { Allowed, NotConfigured, Estop, Comm };

inline ResetDecision decideReset(bool configured, fr_client_library::FaultClass fault)
{
  using fr_client_library::FaultClass;
  if (!configured) {
    return ResetDecision::NotConfigured;
  }
  if (fault == FaultClass::Comm) {
    return ResetDecision::Comm;
  }
  if (fault == FaultClass::Estop) {
    return ResetDecision::Estop;
  }
  return ResetDecision::Allowed;  // also with no fault: ResetAllError is harmless then
}

inline const char * describe(ResetDecision d)
{
  switch (d) {
    case ResetDecision::Allowed: return "reset sent";
    case ResetDecision::NotConfigured: return "hardware is not configured (no connection)";
    case ResetDecision::Estop: return "emergency stop: restart the controller, a reset is refused";
    case ResetDecision::Comm: return "communication fault: reconfigure the hardware first";
  }
  return "?";
}

}  // namespace fr_robot_driver
