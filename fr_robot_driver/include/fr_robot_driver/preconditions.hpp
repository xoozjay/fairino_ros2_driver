// What must hold before the plugin talks to the robot: the checks of configure, activate, the
// gripper's activation and reset_error. The plugin performs the calls and logs; this decides
// (docs/fault_handling.md sections 4-5, sdk_quirks.md F3-F6, G1, G11).
//
// Every check answers "ok, or refuse with this message", so the plugin cannot accidentally forget
// one and the rules can be tested without a controller. The checks never write to the robot: a
// fault is never reset or enabled by the driver, only by the operator.
#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>

#include "fr_client_library/fault_state.hpp"
#include "fr_client_library/robot_api.hpp"  // JointArray
#include "fr_client_library/robot_model.hpp"
#include "fr_client_library/software_version.hpp"
#include "fr_robot_driver/kinematics.hpp"

namespace fr_robot_driver
{
// The answer to a check: `ok`, or refuse with this message (an error, or a warning when `warn`).
struct Verdict
{
  bool ok = true;
  std::string message;   // why it was refused
  bool warn = false;     // ok, but the operator should know (e.g. an unverified version)
};

// ---- configure: what GetSoftwareVersion tells us (sdk_quirks version table, F6) ----

// The controller's software version must be one this driver was checked against; a version outside
// the whitelist is likely to work but is not verified, so it needs an explicit parameter.
Verdict checkVersion(
  const std::optional<fr_client_library::SoftwareVersion> & version, bool allow_unverified)
{
  if (!version) {
    return {false, "GetSoftwareVersion returned an unparseable version", false};
  }
  if (fr_client_library::stateFrameSize(*version) == 0) {
    return {false, "no known state frame size for software version " + fr_client_library::toString(
      *version), false};
  }
  if (!fr_client_library::isVerifiedVersion(*version)) {
    if (!allow_unverified) {
      return {false, "software version " + fr_client_library::toString(*version) +
        " is not verified with this driver (set allow_unverified_version to run anyway)", false};
    }
    return {true, "software version " + fr_client_library::toString(*version) +
      " is NOT verified, running anyway", true};
  }
  return {true, {}, false};
}

// The controller must be the robot this URDF was built for: the limits and the kinematics come from
// fr_description's model.
Verdict checkModel(const std::string & controller_model, const std::string & description_model,
  bool allow_unknown)
{
  if (const char * described = fr_client_library::descriptionModelFor(controller_model)) {
    if (description_model == described) {
      return {true, {}, false};
    }
    return {false, "the controller is a " + controller_model + " (" + described +
      ") but this URDF was built for " + description_model, false};
  }
  if (!allow_unknown) {
    return {false, "controller model '" + controller_model + "' is not one this driver has "
      "kinematics for (allow_unknown_model:=true to run anyway; the flange check still applies)",
      false};
  }
  return {true, "controller model '" + controller_model + "' is unknown, assuming " +
    description_model, true};
}

// ---- activate: the state frame must be trustworthy and the robot ready ----

// The flange pose of the state frame against the forward kinematics of the URDF
// (docs/fault_handling.md section 8). It says nothing about faults: configure checks it so a robot
// that was re-installed while the driver was down is caught before the first motion, and activate
// checks it again.
Verdict checkFlange(const fr_client_library::RobotState384 & state, const Kinematics & kinematics,
  double position_tolerance_mm, double angle_tolerance_deg, const char * model)
{
  std::array<double, 6> q;
  for (int i = 0; i < 6; ++i) {
    q[i] = state.jt_cur_pos[i];
  }
  double flange[6];  // a copy: the frame is packed
  for (int i = 0; i < 6; ++i) {
    flange[i] = state.flange_cur_pos[i];
  }
  const FlangeMismatch m = compareFlange(kinematics, q, flange);
  if (m.position_mm <= position_tolerance_mm && m.angle_deg <= angle_tolerance_deg) {
    return {true, "flange check: " + std::to_string(m.position_mm) + " mm / " +
      std::to_string(m.angle_deg) + " deg from FK (limits " +
      std::to_string(position_tolerance_mm) + " mm / " + std::to_string(angle_tolerance_deg) +
      " deg)", false};
  }
  return {false, "flange check failed: the controller's flange pose is " +
    std::to_string(m.position_mm) + " mm / " + std::to_string(m.angle_deg) +
    " deg away from the forward kinematics of " + std::string(model) + " (limits " +
    std::to_string(position_tolerance_mm) + " mm / " + std::to_string(angle_tolerance_deg) +
    " deg): wrong robot model, joint zero offsets or installation? (fk_check:=false to disable)",
    false};
}

// Never reset, never enable (F3, F5): the frame must be fresh and fault-free, and the flange pose
// must agree with the kinematics.
Verdict checkActivatable(const fr_client_library::RobotState384 & state, bool fresh,
  const Kinematics * kinematics, double position_tolerance_mm, double angle_tolerance_deg,
  const char * model)
{
  if (!fresh) {
    return {false, "no fresh state frame", false};
  }
  const fr_client_library::FaultClass fc = fr_client_library::classifyState(state);
  if (fc != fr_client_library::FaultClass::None) {
    return {false, std::string(fr_client_library::toString(fc)) + ": " +
      fr_client_library::describeFault(state), false};
  }
  if (!kinematics) {
    return {true, {}, false};
  }
  return checkFlange(state, *kinematics, position_tolerance_mm, angle_tolerance_deg, model);
}

// ---- the gripper's activation: only the operator, never the driver (sdk_quirks G1, G2, G11) ----

// Activating opens the gripper to its full stroke whatever it holds (G2), so it needs an explicit
// request. A gripper fault is no reason to refuse: activating is what clears it (G11).
std::string refuseGripperActivation(
  bool configured, const fr_client_library::RobotState384 * state, bool fresh, bool protective_stop,
  bool gripper_active, const std::array<double, 6> & joint_velocity_deg_s, double still_deg_s,
  bool servo_streaming)
{
  if (!configured) {
    return "hardware not configured";
  }
  if (!state || !fresh) {
    return "no fresh state frame";
  }
  if (const fr_client_library::FaultClass fc = fr_client_library::classifyState(*state);
    fc != fr_client_library::FaultClass::None)
  {
    return std::string(fr_client_library::toString(fc)) + ": " +
      fr_client_library::describeFault(*state);
  }
  if (protective_stop) {
    return "a command was refused by the step check";
  }
  if (gripper_active) {
    return "the gripper is already active";
  }
  for (const double v : joint_velocity_deg_s) {
    if (std::abs(v) > still_deg_s) {
      return "the arm is moving";
    }
  }
  if (servo_streaming) {
    return "the arm is being commanded (servo stream running)";
  }
  return {};
}

// ---- reset_error: when may the operator's request be forwarded? (F3, F4) ----

enum class ResetDecision { Allowed, NotConfigured, Estop, Comm };

// The operator may reset a robot fault. An emergency stop can only be cleared by restarting the
// controller (F4), and a communication fault means the link cannot be trusted at all.
ResetDecision decideReset(bool configured, fr_client_library::FaultClass fault)
{
  if (!configured) {
    return ResetDecision::NotConfigured;
  }
  if (fault == fr_client_library::FaultClass::Comm) {
    return ResetDecision::Comm;
  }
  if (fault == fr_client_library::FaultClass::Estop) {
    return ResetDecision::Estop;
  }
  return ResetDecision::Allowed;  // also with no fault: ResetAllError is harmless then
}

const char * describe(ResetDecision d)
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
