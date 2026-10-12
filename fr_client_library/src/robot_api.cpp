#include "fr_client_library/robot_api.hpp"

#include <algorithm>
#include <stdexcept>

namespace fr_client_library
{
namespace
{
ApiStatus error(ApiError e, int code = 0, std::string message = {})
{
  return ApiStatus{e, code, std::move(message)};
}
}  // namespace

ApiStatus RobotApi::call(
  const std::string & method, const std::vector<XmlRpcValue> & params, XmlRpcValue & result)
{
  std::string request;
  try {
    request = encodeMethodCall(method, params);
  } catch (const std::invalid_argument & e) {
    return error(ApiError::InvalidArgument, 0, e.what());  // nothing was sent
  }
  const HttpResult http = http_.post(request);
  switch (http.status) {
    case HttpStatus::Ok: break;
    case HttpStatus::NotConnected: return error(ApiError::NotConnected);
    case HttpStatus::Timeout: return error(ApiError::Timeout);
    case HttpStatus::IoError: return error(ApiError::Io);
    case HttpStatus::ProtocolError: return error(ApiError::Protocol, 0, "bad HTTP response");
  }
  const auto response = decodeMethodResponse(http.body);
  if (!response) {
    http_.close();  // cannot trust the stream any more
    return error(ApiError::Protocol, 0, "bad XML-RPC response to " + method);
  }
  if (response->is_fault) {
    return error(ApiError::Fault, response->fault_code, response->fault_string);
  }
  result = response->value;
  return {};
}

ApiStatus RobotApi::callInt(const std::string & method, const std::vector<XmlRpcValue> & params)
{
  XmlRpcValue result{0};
  if (const ApiStatus st = call(method, params, result); !st.ok()) {
    return st;
  }
  if (!result.isInt()) {
    http_.close();
    return error(ApiError::Protocol, 0, method + ": expected int");
  }
  return result.asInt() == 0 ? ApiStatus{} : error(ApiError::Controller, result.asInt());
}

ApiStatus RobotApi::callArray(
  const std::string & method, const std::vector<XmlRpcValue> & params,
  std::vector<XmlRpcValue> & values)
{
  XmlRpcValue result{0};
  if (const ApiStatus st = call(method, params, result); !st.ok()) {
    return st;
  }
  if (!result.isArray() || result.asArray().empty() || !result.asArray()[0].isInt()) {
    http_.close();
    return error(ApiError::Protocol, 0, method + ": expected [errcode, ...]");
  }
  const auto & a = result.asArray();
  if (a[0].asInt() != 0) {
    return error(ApiError::Controller, a[0].asInt());
  }
  values.assign(a.begin() + 1, a.end());
  return {};
}

ApiResult<RobotVersionInfo> RobotApi::getSoftwareVersion()
{
  ApiResult<RobotVersionInfo> r;
  std::vector<XmlRpcValue> v;
  r.status = callArray("GetSoftwareVersion", {}, v);
  if (!r.ok()) {
    return r;
  }
  if (v.size() < 3 || !v[0].isString() || !v[1].isString() || !v[2].isString()) {
    http_.close();
    r.status = error(ApiError::Protocol, 0, "GetSoftwareVersion: expected 3 strings");
    return r;
  }
  r.value.robot_model = v[0].asString();
  r.value.software_version = v[1].asString();
  r.value.controller_version = v[2].asString();
  r.value.version = parseSoftwareVersion(r.value.software_version);
  return r;
}

ApiResult<int> RobotApi::getRealtimeStateSamplePeriod()
{
  ApiResult<int> r;
  std::vector<XmlRpcValue> v;
  r.status = callArray("GetRobotRealtimeStateSamplePeriod", {}, v);
  if (!r.ok()) {
    return r;
  }
  if (v.empty() || !v[0].isInt()) {
    http_.close();
    r.status = error(ApiError::Protocol, 0, "GetRobotRealtimeStateSamplePeriod: expected int");
    return r;
  }
  r.value = v[0].asInt();
  return r;
}

ApiStatus RobotApi::setRealtimeStateSamplePeriod(int period_ms)
{
  return callInt("SetRobotRealtimeStateSamplePeriod", {period_ms});
}

ApiStatus RobotApi::callDoubles(
  const std::string & method, std::size_t n, std::vector<double> & out)
{
  std::vector<XmlRpcValue> v;
  if (const ApiStatus st = callArray(method, {1}, v); !st.ok()) {
    return st;
  }
  out.clear();
  for (const auto & x : v) {
    if (!x.isDouble()) {break;}
    out.push_back(x.asDouble());
  }
  if (v.size() != n || out.size() != n) {
    http_.close();
    return error(ApiError::Protocol, 0, method + ": expected " + std::to_string(n) + " doubles");
  }
  return {};
}

ApiResult<double> RobotApi::getTargetPayload()
{
  ApiResult<double> r;
  std::vector<double> v;
  r.status = callDoubles("GetTargetPayload", 1, v);
  if (r.ok()) {r.value = v[0];}
  return r;
}

ApiResult<std::array<double, 3>> RobotApi::getTargetPayloadCog()
{
  ApiResult<std::array<double, 3>> r;
  std::vector<double> v;
  r.status = callDoubles("GetTargetPayloadCog", 3, v);
  if (r.ok()) {std::copy(v.begin(), v.end(), r.value.begin());}
  return r;
}

ApiResult<std::array<double, 6>> RobotApi::getTcpOffset()
{
  ApiResult<std::array<double, 6>> r;
  std::vector<double> v;
  r.status = callDoubles("GetTCPOffset", 6, v);
  if (r.ok()) {std::copy(v.begin(), v.end(), r.value.begin());}
  return r;
}

ApiStatus RobotApi::resetAllError() {return callInt("ResetAllError", {});}

ApiStatus RobotApi::servoMoveStart() {return callInt("ServoMoveStart", {});}

ApiStatus RobotApi::servoMoveEnd() {return callInt("ServoMoveEnd", {});}

ApiStatus RobotApi::servoJ(const JointArray & joint_deg, double cmd_t, int id)
{
  XmlRpcValue::Array joints;
  for (const double j : joint_deg) {
    joints.emplace_back(j);
  }
  const XmlRpcValue::Array no_external_axes{0.0, 0.0, 0.0, 0.0};
  // [joints], [external axes], acc, vel, cmdT, filterT, gain, id  (SDK 3.8.4 order)
  return callInt(
    "ServoJ",
    {XmlRpcValue(std::move(joints)), XmlRpcValue(no_external_axes), 0.0, 0.0, cmd_t, 0.0, 0.0, id});
}

}  // namespace fr_client_library
