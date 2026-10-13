// Typed XML-RPC calls (TCP 20003), parameter shapes as in SDK 3.8.4 (robot.cpp).
//
// The SDK checks GetSafetyCode() (SI0/SI1 safety stop, from its state frame) before some motion
// calls and answers 99. This class has no state frame: whoever drives it (the hardware plugin)
// must not send motion while the state frame shows a fault (docs/fault_handling.md).
#pragma once

#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <vector>

#include "fr_client_library/http_connection.hpp"
#include "fr_client_library/software_version.hpp"
#include "fr_client_library/xmlrpc_codec.hpp"

namespace fr_client_library
{
enum class ApiError {
  None,
  NotConnected,
  Timeout,
  Io,          // connection failed or broke; the connection is closed
  Protocol,    // unexpected HTTP / XML / result shape; the connection is closed
  Fault,       // XML-RPC fault; `code` and `message` are the fault's
  Controller,  // the call ran, the controller returned a non-zero error code (`code`)
  InvalidArgument,  // refused before sending (NaN/inf, out of range); nothing was sent
};

using JointArray = std::array<double, 6>;  // degrees

struct ApiStatus
{
  ApiError error = ApiError::None;
  int code = 0;
  std::string message;
  bool ok() const {return error == ApiError::None;}
};

template<typename T>
struct ApiResult
{
  ApiStatus status;
  T value{};
  bool ok() const {return status.ok();}
};

// GetSoftwareVersion returns [errcode, robotModel, webVersion, controllerVersion]. The
// software version is the second string ("webVersion", e.g. "v3.8.4"); the vendor's
// "controllerVersion" ("V3.8.8-LA") is another numbering, kept for display only.
struct RobotVersionInfo
{
  std::string robot_model;
  std::string software_version;    // raw text of webVersion
  std::string controller_version;  // raw, informational
  std::optional<SoftwareVersion> version;  // parsed software_version; nullopt if unparseable
};

class RobotApi
{
public:
  explicit RobotApi(std::string ip, uint16_t port = 20003, HttpOptions options = {})
  : http_(std::move(ip), port, options) {}

  bool connect() {return http_.connect();}
  void close() {http_.close();}
  bool connected() const {return http_.isOpen();}

  ApiResult<RobotVersionInfo> getSoftwareVersion();  // the call's name in the SDK
  ApiResult<int> getRealtimeStateSamplePeriod();  // ms
  ApiStatus setRealtimeStateSamplePeriod(int period_ms);

  // Payload and tool as configured on the controller (read only). Sent with flag = 1, the SDK's
  // default (non-blocking).
  ApiResult<double> getTargetPayload();  // kg
  ApiResult<std::array<double, 3>> getTargetPayloadCog();  // mm
  ApiResult<std::array<double, 6>> getTcpOffset();  // x, y, z [mm], rx, ry, rz [deg]

  // ---- control and motion (every call returns the controller's error code in `status.code`) ----
  // Enabling, mode, speed, payload, tool and digital outputs are left out on purpose: the
  // operator sets them, never the driver (docs/sdk_quirks.md F3, F6, G2).

  ApiStatus resetAllError();  // never call this automatically (docs/fault_handling.md F3)

  // Servo mode: servoMoveStart, a stream of servoJ, servoMoveEnd. Both are empty commands in
  // 3.8.x firmware but are still called as the API prescribes (sdk_quirks S6).
  ApiStatus servoMoveStart();
  ApiStatus servoMoveEnd();
  // `cmd_t` seconds = the period the commands are sent at (whole milliseconds, sdk_quirks S1).
  // acc, vel, filterT and gain are "not opened" in the SDK and sent as 0; no external axes (zeros).
  ApiStatus servoJ(const JointArray & joint_deg, double cmd_t, int id = 0);

  // ---- end-effector gripper (sdk_quirks G1-G7) ----

  // ActGripper(index, 1). Activation opens the gripper to its full stroke whatever it holds (G2):
  // only on an operator's explicit request, never automatically. Returns before the activation is
  // done; the state frame's gripper_active shows when it is (G5). Reset (action 0) is not offered.
  ApiStatus actGripper(int index);
  // MoveGripper(index, pos, vel, force, max_time, block = 1 (non-blocking), type 0 (parallel),
  // rotNum 0.0, rotVel 0, rotTorque 0). pos, vel, force in percent [0, 100] (pos 100 = fully
  // open), max_time in ms [0, 30000]; anything else is refused before sending.
  ApiStatus moveGripper(int index, int position, int velocity, int force, int max_time_ms);

private:
  // Sends the call; on success `result` holds the decoded value.
  ApiStatus call(
    const std::string & method, const std::vector<XmlRpcValue> & params, XmlRpcValue & result);
  // For methods answering a bare errcode.
  ApiStatus callInt(const std::string & method, const std::vector<XmlRpcValue> & params);
  // For getters taking `flag` and answering [errcode, n doubles].
  ApiStatus callDoubles(const std::string & method, std::size_t n, std::vector<double> & out);
  // For methods answering [errcode, values...]; `values` = the rest.
  ApiStatus callArray(
    const std::string & method, const std::vector<XmlRpcValue> & params,
    std::vector<XmlRpcValue> & values);

  HttpConnection http_;
};

}  // namespace fr_client_library
