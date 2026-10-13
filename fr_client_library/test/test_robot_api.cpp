#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <iostream>
#include <map>

#include "fr_client_library/robot_api.hpp"
#include "test_support.hpp"

using namespace fr_client_library;
using namespace fr_test;

namespace
{
HttpOptions fast()
{
  HttpOptions o;
  o.connect_timeout_ms = 500;
  o.request_timeout_ms = 300;
  return o;
}

// Serves one request per entry: remembers the request body, answers with the given XML.
class ScriptedController
{
public:
  explicit ScriptedController(std::vector<std::string> answers)
  : server_([this, answers](int fd) {
        for (const auto & a : answers) {
          const std::string body = readHttpRequest(fd);
          if (body.empty()) {return;}
          requests.push_back(body);
          writeAll(fd, httpResponse(a));
        }
      }) {}
  uint16_t port() const {return server_.port();}
  std::vector<std::string> requests;

private:
  FakeServer server_;
};

const std::string kVersionXml = xmlResponse(
  "<value><array><data><value><i4>0</i4></value><value><string>FR5-V1-002(V6.0)</string></value>"
  "<value><string>v3.8.3</string></value><value><string>V3.8.8-LA</string></value>"
  "</data></array></value>");
}  // namespace

TEST(RobotApi, GetSoftwareVersion)
{
  ScriptedController c({kVersionXml});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const auto r = api.getSoftwareVersion();
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value.robot_model, "FR5-V1-002(V6.0)");
  EXPECT_EQ(r.value.software_version, "v3.8.3");
  EXPECT_EQ(r.value.controller_version, "V3.8.8-LA");  // informational only
  ASSERT_TRUE(r.value.version.has_value());
  EXPECT_EQ(*r.value.version, (SoftwareVersion{3, 8, 3}));
  EXPECT_TRUE(isVerifiedVersion(*r.value.version));
  api.close();
  ASSERT_EQ(c.requests.size(), 1u);
  EXPECT_EQ(
    c.requests[0],
    "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>GetSoftwareVersion</methodName>\r\n"
    "</methodCall>\r\n");
}

TEST(RobotApi, SamplePeriodGetAndSet)
{
  ScriptedController c({
      xmlResponse("<value><array><data><value><i4>0</i4></value><value><i4>8</i4></value></data></array></value>"),
      xmlResponse("<value><i4>0</i4></value>")});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const auto g = api.getRealtimeStateSamplePeriod();
  ASSERT_TRUE(g.ok());
  EXPECT_EQ(g.value, 8);
  EXPECT_TRUE(api.setRealtimeStateSamplePeriod(4).ok());
  api.close();
  ASSERT_EQ(c.requests.size(), 2u);
  EXPECT_NE(c.requests[0].find("GetRobotRealtimeStateSamplePeriod"), std::string::npos);
  EXPECT_EQ(
    c.requests[1],
    "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>SetRobotRealtimeStateSamplePeriod"
    "</methodName>\r\n<params><param><value><i4>4</i4></value></param></params></methodCall>\r\n");
}

// Answers shaped as a 3.8.4.2 controller sends them ([i4 errcode, double...]).
TEST(RobotApi, PayloadAndTcpGetters)
{
  const auto d = [](const char * v) {return std::string("<value><double>") + v + "</double></value>";};
  const auto arr = [](const std::string & values) {
      return xmlResponse("<value><array><data><value><i4>0</i4></value>" + values +
               "</data></array></value>");
    };
  ScriptedController c({
      arr(d("2.5")),
      arr(d("3") + d("4") + d("5")),
      arr(d("1") + d("2") + d("3") + d("-90") + d("0.5") + d("180")),
      arr(d("1") + d("2"))});  // too short
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const auto w = api.getTargetPayload();
  ASSERT_TRUE(w.ok());
  EXPECT_DOUBLE_EQ(w.value, 2.5);
  const auto cog = api.getTargetPayloadCog();
  ASSERT_TRUE(cog.ok());
  EXPECT_EQ(cog.value, (std::array<double, 3>{3, 4, 5}));
  const auto tcp = api.getTcpOffset();
  ASSERT_TRUE(tcp.ok());
  EXPECT_EQ(tcp.value, (std::array<double, 6>{1, 2, 3, -90, 0.5, 180}));
  EXPECT_EQ(api.getTcpOffset().status.error, ApiError::Protocol);
  api.close();
  ASSERT_EQ(c.requests.size(), 4u);
  EXPECT_EQ(
    c.requests[0],
    "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>GetTargetPayload</methodName>\r\n"
    "<params><param><value><i4>1</i4></value></param></params></methodCall>\r\n");
  EXPECT_NE(c.requests[1].find("GetTargetPayloadCog"), std::string::npos);
  EXPECT_NE(c.requests[2].find("GetTCPOffset"), std::string::npos);
}

TEST(RobotApi, ControllerErrorCode)
{
  ScriptedController c({
      xmlResponse("<value><array><data><value><i4>14</i4></value></data></array></value>"),
      xmlResponse("<value><i4>3</i4></value>")});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const auto g = api.getSoftwareVersion();
  EXPECT_EQ(g.status.error, ApiError::Controller);
  EXPECT_EQ(g.status.code, 14);
  const auto s = api.setRealtimeStateSamplePeriod(4);
  EXPECT_EQ(s.error, ApiError::Controller);
  EXPECT_EQ(s.code, 3);
  EXPECT_TRUE(api.connected());  // a controller error is not a transport error
}

TEST(RobotApi, XmlRpcFault)
{
  ScriptedController c({
      "<?xml version=\"1.0\"?><methodResponse><fault><value><struct>"
      "<member><name>faultCode</name><value><i4>-503</i4></value></member>"
      "<member><name>faultString</name><value><string>Method not defined</string></value></member>"
      "</struct></value></fault></methodResponse>"});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const auto r = api.getSoftwareVersion();
  EXPECT_EQ(r.status.error, ApiError::Fault);
  EXPECT_EQ(r.status.code, -503);
  EXPECT_EQ(r.status.message, "Method not defined");
}

TEST(RobotApi, UnexpectedShapeClosesTheConnection)
{
  ScriptedController c({xmlResponse("<value><i4>0</i4></value>")});  // not an array
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  EXPECT_EQ(api.getSoftwareVersion().status.error, ApiError::Protocol);
  EXPECT_FALSE(api.connected());
}

TEST(RobotApi, GarbageBodyIsProtocolError)
{
  ScriptedController c({"this is not xml"});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  EXPECT_EQ(api.getRealtimeStateSamplePeriod().status.error, ApiError::Protocol);
  EXPECT_FALSE(api.connected());
}

TEST(RobotApi, NotConnected)
{
  RobotApi api("127.0.0.1", 1, fast());
  EXPECT_EQ(api.getSoftwareVersion().status.error, ApiError::NotConnected);
}

TEST(RobotApi, UnparseableSoftwareVersionIsReportedNotGuessed)
{
  ScriptedController c({xmlResponse(
      "<value><array><data><value><i4>0</i4></value><value><string>FR5</string></value>"
      "<value><string>V3.8.8-LA</string></value><value><string>v3.8.3</string></value>"
      "</data></array></value>")});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const auto r = api.getSoftwareVersion();
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value.version.has_value());  // second string is not a version: no fallback
}

// ---- control and motion calls: bodies compared with what the SDK 3.8.4 sends (robot.cpp) ----

namespace
{
std::string I(int v) {return "<param><value><i4>" + std::to_string(v) + "</i4></value></param>";}
std::string D(const char * v) {return std::string("<param><value><double>") + v + "</double></value></param>";}
std::string A(const std::string & inner)
{
  return "<param><value><array><data>" + inner + "</data></array></value></param>";
}
std::string dv(const char * v) {return std::string("<value><double>") + v + "</double></value>";}
std::string call(const std::string & method, const std::string & params = "")
{
  return "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>" + method + "</methodName>\r\n" +
         (params.empty() ? "" : "<params>" + params + "</params>") + "</methodCall>\r\n";
}
const std::string kOk = xmlResponse("<value><i4>0</i4></value>");
}  // namespace

TEST(RobotApiControl, RequestsMatchTheSdk)
{
  ScriptedController c(std::vector<std::string>(4, kOk));
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());

  EXPECT_TRUE(api.resetAllError().ok());
  EXPECT_TRUE(api.servoMoveStart().ok());
  EXPECT_TRUE(api.servoMoveEnd().ok());
  EXPECT_TRUE(api.servoJ({1.5, -2, 3, 4, 5, 6.25}, 0.008, 7).ok());
  api.close();

  const auto & r = c.requests;
  ASSERT_EQ(r.size(), 4u);
  EXPECT_EQ(r[0], call("ResetAllError"));
  EXPECT_EQ(r[1], call("ServoMoveStart"));
  EXPECT_EQ(r[2], call("ServoMoveEnd"));
  // ServoJ: [joints], [external axes], acc, vel, cmdT, filterT, gain, id
  EXPECT_EQ(
    r[3],
    call(
      "ServoJ",
      A(dv("1.500000") + dv("-2.000000") + dv("3.000000") + dv("4.000000") + dv("5.000000") +
      dv("6.250000")) +
      A(dv("0.000000") + dv("0.000000") + dv("0.000000") + dv("0.000000")) +
      D("0.000000") + D("0.000000") + D("0.008000") + D("0.000000") + D("0.000000") + I(7)));
}

TEST(RobotApiControl, GripperRequestsMatchTheSdk)
{
  ScriptedController c({kOk, kOk});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  EXPECT_TRUE(api.actGripper(1).ok());
  EXPECT_TRUE(api.moveGripper(1, 50, 20, 30, 5000).ok());
  api.close();

  const auto & r = c.requests;
  ASSERT_EQ(r.size(), 2u);
  EXPECT_EQ(r[0], call("ActGripper", I(1) + I(1)));
  // index, pos, vel, force, max_time, block (1 = non-blocking), type, rotNum (double), rotVel, rotTorque
  EXPECT_EQ(
    r[1], call("MoveGripper", I(1) + I(50) + I(20) + I(30) + I(5000) + I(1) + I(0) + D("0.000000") +
    I(0) + I(0)));
}

TEST(RobotApiControl, GripperArgumentsOutOfRangeAreRefusedBeforeSending)
{
  ScriptedController c({});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  EXPECT_EQ(api.moveGripper(1, 101, 50, 20, 1000).error, ApiError::InvalidArgument);
  EXPECT_EQ(api.moveGripper(1, 50, -1, 20, 1000).error, ApiError::InvalidArgument);
  EXPECT_EQ(api.moveGripper(1, 50, 50, 120, 1000).error, ApiError::InvalidArgument);
  EXPECT_EQ(api.moveGripper(1, 50, 50, 20, 30001).error, ApiError::InvalidArgument);
  EXPECT_TRUE(api.connected());
}

TEST(RobotApiControl, ControllerErrorCodeIsReported)
{
  ScriptedController c({xmlResponse("<value><i4>112</i4></value>")});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const auto st = api.servoJ({0, 0, 0, 0, 0, 0}, 0.008);
  EXPECT_EQ(st.error, ApiError::Controller);
  EXPECT_EQ(st.code, 112);
  EXPECT_TRUE(api.connected());
}

TEST(RobotApiControl, BadArgumentsAreRefusedBeforeSending)
{
  ScriptedController c({});
  RobotApi api("127.0.0.1", c.port(), fast());
  ASSERT_TRUE(api.connect());
  const double nan = std::nan("");
  EXPECT_EQ(api.servoJ({0, 0, nan, 0, 0, 0}, 0.008).error, ApiError::InvalidArgument);
  EXPECT_EQ(api.servoJ({0, 0, 0, 0, 0, 0}, nan).error, ApiError::InvalidArgument);
  EXPECT_EQ(api.servoJ({0, 0, 0, std::numeric_limits<double>::infinity(), 0, 0}, 0.008).error,
    ApiError::InvalidArgument);
  EXPECT_TRUE(api.connected());   // nothing was sent, the connection is fine
  EXPECT_TRUE(c.requests.empty());
}

// Opt-in: FR_SIM_IP=192.168.58.2 (writes the sample period back to its current value)
TEST(Simulator, ReadVersionAndPeriod)
{
  const char * ip = std::getenv("FR_SIM_IP");
  if (ip == nullptr) {
    GTEST_SKIP() << "FR_SIM_IP not set";
  }
  RobotApi api(ip);
  ASSERT_TRUE(api.connect());
  const auto v = api.getSoftwareVersion();
  ASSERT_TRUE(v.ok()) << v.status.message;
  std::cout << "model=" << v.value.robot_model << " software=" << v.value.software_version
            << " controller=" << v.value.controller_version << "\n";
  ASSERT_TRUE(v.value.version.has_value());
  EXPECT_TRUE(isVerifiedVersion(*v.value.version));
  EXPECT_EQ(stateFrameSize(*v.value.version), kRobotState384Size);

  const auto p = api.getRealtimeStateSamplePeriod();
  ASSERT_TRUE(p.ok());
  EXPECT_TRUE(api.setRealtimeStateSamplePeriod(p.value).ok());
  const auto p2 = api.getRealtimeStateSamplePeriod();
  ASSERT_TRUE(p2.ok());
  EXPECT_EQ(p2.value, p.value);

  // more requests than one connection serves (max=30): rotation must be invisible
  for (int i = 0; i < 70; ++i) {
    ASSERT_TRUE(api.getRealtimeStateSamplePeriod().ok()) << "request " << i;
  }
}
