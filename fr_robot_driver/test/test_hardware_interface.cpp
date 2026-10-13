// State machine of FrRobotHardwareInterface against a fake controller, driven through the
// ResourceManager exactly as the controller manager does (docs/fault_handling.md).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "fake_controller.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using hardware_interface::return_type;
namespace lc = lifecycle_msgs::msg;

namespace
{
constexpr double kDeg = M_PI / 180.0;

std::string run(const std::string & cmd)
{
  std::string out;
  FILE * p = ::popen(cmd.c_str(), "r");
  char buf[4096];
  while (p && std::fgets(buf, sizeof(buf), p)) {out += buf;}
  if (p) {::pclose(p);}
  return out;
}

// The real xacro, so the description in these tests is the one the launch file produces.
std::string urdf(const fr_test::FakeController & c, const std::string & extra_params = "",
  int update_rate_hz = 125)
{
  const std::string share = ament_index_cpp::get_package_share_directory("fr_robot_driver");
  std::string xml = run("xacro " + share + "/urdf/fr_controlled.urdf.xacro model:=fairino5_v6 "
      "use_mock_hardware:=false ip:=127.0.0.1 update_rate_hz:=" + std::to_string(update_rate_hz) +
      " 2>/dev/null");
  const std::string ports = "<param name=\"xmlrpc_port\">" + std::to_string(c.xmlrpcPort()) +
    "</param><param name=\"state_port\">" + std::to_string(c.statePort()) +
    "</param><param name=\"instant_port\">" + std::to_string(c.instantPort()) + "</param>" + extra_params;
  const auto at = xml.find("</hardware>");
  return xml.substr(0, at) + ports + xml.substr(at);
}

// One robot: fake controller + ResourceManager on its own executor (for the reset service).
struct Rig
{
  // multi_threaded: the plugin's callbacks (status timer, services) run in parallel to the
  // lifecycle calls, as under ros2_control_node
  explicit Rig(const std::string & extra = "", int update_rate_hz = 125,
    bool multi_threaded = false)
  : update_rate(update_rate_hz)
  {
    if (multi_threaded) {
      executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    } else {
      executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf(fake, extra, update_rate_hz);
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    params.logger = rclcpp::get_logger("test");
    params.executor = executor;
    params.update_rate = update_rate_hz;
    rm = std::make_unique<hardware_interface::ResourceManager>(params, true);
    status_node = std::make_shared<rclcpp::Node>("status_listener");
    status_sub = status_node->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "/fr_robot_hw/status", rclcpp::QoS(1).transient_local(),
      [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(status_mutex);
        last_status = *m;
        ++status_count;
      });
    executor->add_node(status_node);
    // Spin only now, as controller_manager does: a component that fails on_init is destroyed inside
    // the ResourceManager constructor together with the node its base class added to the executor,
    // and destroying a node while another thread waits on it crashes (about 1 in 50 runs under load).
    spinner = std::thread([this] {executor->spin();});
  }
  ~Rig()
  {
    // Stop spinning before the component's node goes away: destroying it under a running
    // executor crashes in rmw_wait. cancel() before spin() has started is a no-op and spin() would
    // then never return, so wait for it first (a short test gets here within microseconds).
    while (!executor->is_spinning()) {std::this_thread::yield();}
    executor->cancel();
    spinner.join();
    rm.reset();
  }

  int stateId() {return rm->get_components_status().at("fr_robot").state.id();}
  bool go(uint8_t id, const char * label)
  {
    rclcpp_lifecycle::State s(id, label);
    return rm->set_component_state("fr_robot", s) == return_type::OK;
  }
  bool configure() {return go(lc::State::PRIMARY_STATE_INACTIVE, "inactive");}
  bool activate() {return go(lc::State::PRIMARY_STATE_ACTIVE, "active");}

  // one controller cycle: read, (caller's commands), write; the first non-OK status is returned
  hardware_interface::HardwareReadWriteStatus cycle(
    const std::function<void()> & commands = {})
  {
    const rclcpp::Time now = params_clock.now();
    const auto period = rclcpp::Duration::from_seconds(1.0 / update_rate);
    auto status = rm->read(now, period);
    if (status.result == return_type::OK) {   // after a failed read the component is gone
      if (commands) {commands();}
      status = rm->write(now, period);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1000000 / update_rate));
    return status;
  }
  void run(int cycles) {for (int i = 0; i < cycles; ++i) {cycle();}}

  double state(const std::string & name)
  {
    auto s = rm->claim_state_interface(name);
    return s.get_optional<double>().value_or(std::nan(""));
  }
  bool available(const std::string & name) {return rm->state_interface_is_available(name);}

  // Calls ~/reset_error like an operator; returns (success, message).
  std::pair<bool, std::string> reset()
  {
    auto node = std::make_shared<rclcpp::Node>("reset_caller");
    auto client = node->create_client<std_srvs::srv::Trigger>("/fr_robot_hw/reset_error");
    if (!client->wait_for_service(2s)) {return {false, "service not available"};}
    auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    if (ex.spin_until_future_complete(fut, 3s) != rclcpp::FutureReturnCode::SUCCESS) {
      return {false, "no reply"};
    }
    const auto response = fut.get();
    return {response->success, response->message};
  }

  // ---- the `~/status` topic ----
  diagnostic_msgs::msg::DiagnosticStatus status()
  {
    std::lock_guard<std::mutex> l(status_mutex);
    return last_status;
  }
  static std::string value(const diagnostic_msgs::msg::DiagnosticStatus & s, const std::string & key)
  {
    for (const auto & kv : s.values) {
      if (kv.key == key) {return kv.value;}
    }
    return "<missing>";
  }
  // Waits (without running controller cycles) until the status satisfies `pred`.
  bool waitStatus(
    const std::function<bool(const diagnostic_msgs::msg::DiagnosticStatus &)> & pred,
    std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred(status())) {return true;}
      std::this_thread::sleep_for(20ms);
    }
    return pred(status());
  }

  fr_test::FakeController fake;
  std::mutex status_mutex;
  diagnostic_msgs::msg::DiagnosticStatus last_status;
  int status_count = 0;
  rclcpp::Node::SharedPtr status_node;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_sub;
  int update_rate;
  rclcpp::Clock params_clock{RCL_STEADY_TIME};
  rclcpp::Executor::SharedPtr executor;
  std::thread spinner;
  std::unique_ptr<hardware_interface::ResourceManager> rm;
};

const char * kPan = "shoulder_pan_joint/position";
}  // namespace

TEST(HardwareInterface, ConfigureReadsTheRobotAndExportsItsState)
{
  Rig r;
  ASSERT_EQ(r.stateId(), lc::State::PRIMARY_STATE_UNCONFIGURED);
  r.fake.edit([](auto & s) {s.mc_queue_len = 7; s.main_code = 0;});
  ASSERT_TRUE(r.configure());           // software version v3.8.4.2 counts as 3.8.4
  r.run(5);
  EXPECT_EQ(r.fake.count("GetSoftwareVersion"), 1);
  EXPECT_NEAR(r.state(kPan), 0.0, 1e-9);
  EXPECT_NEAR(r.state("shoulder_lift_joint/position"), -90 * kDeg, 1e-9);   // deg -> rad
  EXPECT_NEAR(r.state("elbow_joint/position"), 90 * kDeg, 1e-9);
  EXPECT_EQ(r.state("fr_status/fault_class"), 0.0);
  EXPECT_FALSE(r.rm->state_interface_exists("fr_status/main_code"));   // the rest is in ~/status
  ASSERT_TRUE(r.waitStatus([](const auto & s) {
      return Rig::value(s, "enabled") == "1" && Rig::value(s, "servo_queue_len") == "7";}));
}

TEST(HardwareInterface, ActivationStartsAtTheActualPositionAndStreamsCommands)
{
  Rig r;
  r.fake.edit([](auto & s) {s.jt_cur_pos[0] = 12.5;});
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());           // no read() in between: startup case
  ASSERT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(r.fake.count("ServoMoveStart"), 1);

  auto cmd = r.rm->claim_command_interface(kPan);
  EXPECT_NEAR(cmd.get_optional<double>().value_or(-1), 12.5 * kDeg, 1e-9);  // command := actual
  for (int i = 0; i < 20; ++i) {
    r.cycle([&] {(void)cmd.set_value(12.5 * kDeg + 0.001 * i);});
  }
  const auto sent = r.fake.servoJ();
  ASSERT_GE(sent.size(), 20u);
  EXPECT_NEAR(sent.front()[0], 12.5, 1e-6);            // the prefill stands still at the actual position
  EXPECT_NEAR(sent.back()[0], 12.5 + 0.001 * 19 / kDeg * 1.0, 0.1);   // and the stream follows the command
}

TEST(HardwareInterface, UnverifiedSoftwareVersionIsRefusedUnlessAllowed)
{
  {
    Rig r;
    r.fake.setVersion("v3.8.5");             // known frame size, but not verified
    EXPECT_FALSE(r.configure());
    EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_UNCONFIGURED);
  }
  {
    Rig r("<param name=\"allow_unverified_version\">true</param>");
    r.fake.setVersion("v3.8.5");
    EXPECT_TRUE(r.configure());
  }
  {
    Rig r("<param name=\"allow_unverified_version\">true</param>");
    r.fake.setVersion("v3.9.9");             // no state frame size known: refused even so
    EXPECT_FALSE(r.configure());
  }
  {
    Rig r;
    r.fake.setVersion("V3.8.8-LA");          // the vendor's controller string is not a software version
    EXPECT_FALSE(r.configure());
  }
}

// ---- robot model and flange (forward kinematics) checks ----

TEST(HardwareInterface, AControllerOfAnotherModelIsRefused)
{
  Rig r;                                     // the URDF is for fairino5_v6
  r.fake.setModel("FR20-V1-001(V6.0)");      // a known model, but not this one
  EXPECT_FALSE(r.configure());
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(r.fake.count("SetRobotRealtimeStateSamplePeriod"), 0);   // refused before touching anything
}

TEST(HardwareInterface, AnUnknownModelNeedsExplicitPermission)
{
  {
    Rig r;
    r.fake.setModel("FR5-V1-001(V5.0)");     // the V5.0 generation: no kinematics for it here
    EXPECT_FALSE(r.configure());
  }
  {
    Rig r("<param name=\"allow_unknown_model\">true</param>");
    r.fake.setModel("FR5-V1-001(V5.0)");
    EXPECT_TRUE(r.configure());              // the flange check still guards it
  }
  {
    Rig r("<param name=\"allow_unknown_model\">true</param>");
    r.fake.setModel("FR5-V1-001(V5.0)");
    r.fake.flange_error_mm = 80;
    EXPECT_FALSE(r.configure());             // allowed to be unknown, not allowed to disagree
  }
}

TEST(HardwareInterface, AFlangePoseThatDisagreesWithTheKinematicsIsRefused)
{
  {
    Rig r;
    r.fake.flange_error_mm = 50;             // e.g. wrong zero offsets or installation
    EXPECT_FALSE(r.configure());
    EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_UNCONFIGURED);
  }
  {
    Rig r;
    r.fake.flange_error_mm = 9;              // inside the default 10 mm
    EXPECT_TRUE(r.configure());
  }
  {
    Rig r("<param name=\"fk_check\">false</param>");
    r.fake.flange_error_mm = 500;
    EXPECT_TRUE(r.configure());              // can be switched off
  }
  {
    Rig r("<param name=\"fk_position_tolerance_mm\">100</param>");
    r.fake.flange_error_mm = 50;
    EXPECT_TRUE(r.configure());              // or loosened
  }
}

TEST(HardwareInterface, ActivationChecksTheFlangeAgain)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  r.run(3);
  r.fake.flange_error_mm = 50;               // e.g. the robot was re-installed meanwhile
  r.run(3);
  EXPECT_FALSE(r.activate());
  EXPECT_EQ(r.fake.count("ServoMoveStart"), 0);
  r.fake.flange_error_mm = 0;
  r.run(3);
  EXPECT_TRUE(r.activate());
}

TEST(HardwareInterface, ActivationIsRefusedWhileAFaultIsPresent)
{
  const std::vector<std::pair<const char *, std::function<void(fr_client_library::RobotState384 &)>>> cases = {
    {"robot fault", [](auto & s) {s.main_code = 1; s.sub_code = 23;}},
    {"emergency stop", [](auto & s) {s.EmergencyStop = 1;}},
    {"collision", [](auto & s) {s.collisionState = 1;}},
    {"safety stop", [](auto & s) {s.safety_stop0_state = 1;}},
    {"not enabled", [](auto & s) {s.rbtEnableState = 0;}},
  };
  for (const auto & [name, inject] : cases) {
    Rig r;
    r.fake.edit(inject);
    ASSERT_TRUE(r.configure()) << name;
    r.run(3);
    EXPECT_FALSE(r.activate()) << name;                     // never resets, never enables
    EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_INACTIVE) << name;
    EXPECT_EQ(r.fake.count("ServoMoveStart"), 0) << name;
    EXPECT_EQ(r.fake.count("ResetAllError"), 0) << name;
  }
}

TEST(HardwareInterface, RobotFaultStopsTheStreamAndRecoversOnlyByTheOperator)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
  const auto before = r.fake.servoJ().size();

  r.fake.edit([](auto & s) {s.main_code = 1; s.sub_code = 23;});
  bool deactivated = false;
  for (int i = 0; i < 30 && !deactivated; ++i) {
    deactivated = r.cycle().result == return_type::DEACTIVATE;
  }
  ASSERT_TRUE(deactivated);
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_INACTIVE);   // not UNCONFIGURED: the link is fine
  r.run(10);
  const auto frozen = r.fake.servoJ().size();
  r.run(20);
  EXPECT_EQ(r.fake.servoJ().size(), frozen);                   // nothing is sent while faulted
  EXPECT_LT(frozen, before + 10);
  EXPECT_EQ(r.state("fr_status/fault_class"), 1.0);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {                // codes reported as they are
      return Rig::value(s, "main_code") == "1" && Rig::value(s, "sub_code") == "23";}));
  EXPECT_EQ(r.fake.count("ResetAllError"), 0);                 // F3: never by itself
  EXPECT_EQ(r.fake.stops(), 0);                                // the firmware stops by itself

  EXPECT_FALSE(r.activate());                                  // still faulted
  const auto [ok, message] = r.reset();                        // operator
  EXPECT_TRUE(ok) << message;
  EXPECT_EQ(r.fake.count("ResetAllError"), 1);
  r.run(5);
  EXPECT_EQ(r.state("fr_status/fault_class"), 0.0);
  EXPECT_TRUE(r.activate());
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);
}

TEST(HardwareInterface, EmergencyStopRefusesAnyReset)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(5);
  r.fake.edit([](auto & s) {s.EmergencyStop = 1;});
  bool deactivated = false;
  for (int i = 0; i < 30 && !deactivated; ++i) {
    deactivated = r.cycle().result == return_type::DEACTIVATE;
  }
  ASSERT_TRUE(deactivated);
  EXPECT_EQ(r.state("fr_status/fault_class"), 2.0);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "estop") == "1";}));
  const auto [ok, message] = r.reset();
  EXPECT_FALSE(ok);
  EXPECT_NE(message.find("emergency stop"), std::string::npos) << message;
  EXPECT_EQ(r.fake.count("ResetAllError"), 0);   // F4: restart the controller instead
  EXPECT_FALSE(r.activate());
}

TEST(HardwareInterface, SilentStateStreamIsACommunicationFault)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(5);
  r.fake.silent = true;                     // connection stays, frames stop
  bool errored = false;
  for (int i = 0; i < 40 && !errored; ++i) {
    errored = r.cycle().result == return_type::ERROR;
  }
  ASSERT_TRUE(errored);
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_UNCONFIGURED);   // -> on_error
  const auto [ok, message] = r.reset();
  EXPECT_FALSE(ok);                         // no reset on an untrustworthy link
  EXPECT_EQ(r.fake.count("ResetAllError"), 0);
}

TEST(HardwareInterface, LostStateConnectionIsACommunicationFault)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(5);
  r.fake.drop_state = true;
  bool errored = false;
  for (int i = 0; i < 40 && !errored; ++i) {
    errored = r.cycle().result == return_type::ERROR;
  }
  EXPECT_TRUE(errored);
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_UNCONFIGURED);
}

// ---- the status topic (docs/fault_handling.md section 7) ----

TEST(StatusTopic, IsPublishedInEveryLifecycleStateAndFollowsFaults)
{
  Rig r;
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return !s.values.empty();}));   // unconfigured: no connection yet
  EXPECT_EQ(Rig::value(r.status(), "hardware_state"), "unconfigured");
  EXPECT_EQ(Rig::value(r.status(), "link"), "down");

  ASSERT_TRUE(r.configure());
  r.run(5);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "hardware_state") == "inactive";}));
  EXPECT_EQ(Rig::value(r.status(), "link"), "up");
  EXPECT_EQ(r.status().level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ASSERT_TRUE(r.activate());
  r.run(5);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "hardware_state") == "active";}));
  EXPECT_EQ(Rig::value(r.status(), "fault_class"), "0");

  r.fake.edit([](auto & s) {s.main_code = 1; s.sub_code = 23;});
  for (int i = 0; i < 30 && r.stateId() == lc::State::PRIMARY_STATE_ACTIVE; ++i) {r.cycle();}
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "fault_class") == "1";}));
  EXPECT_EQ(Rig::value(r.status(), "main_code"), "1");
  EXPECT_EQ(Rig::value(r.status(), "sub_code"), "23");
  EXPECT_EQ(Rig::value(r.status(), "hardware_state"), "inactive");
  EXPECT_EQ(r.status().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(StatusTopic, LinkLossThenTheControllersOwnFaultIsSeenWithoutAnyCommand)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(5);

  r.fake.drop_state = true;                            // the controller vanishes (power cut)
  bool errored = false;
  for (int i = 0; i < 40 && !errored; ++i) {
    errored = r.cycle().result == return_type::ERROR;
  }
  ASSERT_TRUE(errored);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "fault_class") == "3";}));
  const auto down = r.status();
  EXPECT_EQ(Rig::value(down, "link"), "down");
  EXPECT_EQ(Rig::value(down, "hardware_state"), "unconfigured");
  EXPECT_NE(down.message.find("interrupted"), std::string::npos) << down.message;
  EXPECT_NE(Rig::value(down, "since"), "<missing>");

  // From here on the driver may only watch the state stream: nothing else is touched.
  std::this_thread::sleep_for(500ms);                  // the cleanup thread has closed everything
  const auto calls = r.fake.xmlrpcCalls();
  const auto servoj = r.fake.servoJ().size();
  const int xmlrpc_conn = r.fake.xmlrpcConnections(), instant_conn = r.fake.instantConnections();
  const int stops = r.fake.stops(), states = r.fake.stateConnections();

  r.fake.edit([](auto & s) {s.main_code = 1; s.sub_code = 149; s.safety_stop0_state = 1; s.rbtEnableState = 0;});
  r.fake.drop_state = false;                           // the controller is back and reports its fault
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "fault_class") == "1";}, 6s));
  const auto fault = r.status();
  EXPECT_EQ(Rig::value(fault, "main_code"), "1");
  EXPECT_EQ(Rig::value(fault, "sub_code"), "149");
  EXPECT_EQ(Rig::value(fault, "link"), "up");
  EXPECT_EQ(Rig::value(fault, "hardware_state"), "unconfigured");   // moving again stays the operator's call
  EXPECT_NE(Rig::value(fault, "previous").find("communication fault"), std::string::npos)
    << Rig::value(fault, "previous");
  EXPECT_GT(r.fake.stateConnections(), states);        // it reconnected the state stream...

  EXPECT_EQ(r.fake.xmlrpcCalls(), calls);              // ...and nothing else: no XML-RPC,
  EXPECT_EQ(r.fake.xmlrpcConnections(), xmlrpc_conn);
  EXPECT_EQ(r.fake.instantConnections(), instant_conn);  // no 8080,
  EXPECT_EQ(r.fake.servoJ().size(), servoj);           // no ServoJ,
  EXPECT_EQ(r.fake.stops(), stops);
  EXPECT_EQ(r.fake.count("ResetAllError"), 0);         // and never a reset

  // The operator can still configure again (the monitor lets go of the state stream).
  r.fake.edit([](auto & s) {s.main_code = 0; s.sub_code = 0; s.safety_stop0_state = 0; s.rbtEnableState = 1;});
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(5);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {
      return Rig::value(s, "hardware_state") == "active" && Rig::value(s, "fault_class") == "0";}));
}

TEST(HardwareInterface, StepCheckRefusesAJumpAndTheFirmwareNeverSeesIt)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  auto cmd = r.rm->claim_command_interface(kPan);
  r.run(10);
  // the FR5's URDF limit is 3.15 rad/s = 180 deg/s: 1.44 deg per 8 ms step
  bool deactivated = false;
  for (int i = 0; i < 5 && !deactivated; ++i) {
    deactivated = r.cycle([&] {(void)cmd.set_value(30 * kDeg);}).result == return_type::DEACTIVATE;
  }
  ASSERT_TRUE(deactivated);
  for (const auto & t : r.fake.servoJ()) {
    EXPECT_LT(std::abs(t[0]), 2.0);               // 30 deg never went out
  }
  r.run(3);
  EXPECT_EQ(r.state("fr_status/fault_class"), 1.0);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {   // our own protective stop, not the firmware's
      return Rig::value(s, "fault_class") == "1" && Rig::value(s, "main_code") == "0";}));
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_INACTIVE);
}

TEST(HardwareInterface, ASmoothRampWithinTheLimitIsAccepted)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  auto cmd = r.rm->claim_command_interface(kPan);
  for (int i = 1; i <= 100; ++i) {                  // 1 deg per step: within the 1.44 deg limit
    ASSERT_NE(r.cycle([&] {(void)cmd.set_value(i * kDeg);}).result, return_type::DEACTIVATE) << i;
  }
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);
  // the sender runs a few segments behind the producer (the resampling lead)
  for (int i = 0; i < 100 && std::abs(r.fake.servoJ().back()[0] - 100.0) > 1e-6; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_NEAR(r.fake.servoJ().back()[0], 100.0, 1e-6);
}

TEST(HardwareInterface, ControllerFasterThanCmdTSendsEveryRatioThCycle)
{
  Rig r("", 500);                                   // 500 Hz controller, cmdT 8 ms: one segment per 4 cycles
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  auto cmd = r.rm->claim_command_interface(kPan);
  const auto base = r.fake.servoJ().size();         // the prefill
  for (int i = 1; i <= 400; ++i) {                  // 0.8 s of a slow ramp (a standing arm pauses)
    r.cycle([&] {(void)cmd.set_value(i * 0.0001);});
  }
  const auto sent = r.fake.servoJ().size() - base;
  EXPECT_GE(sent, 85u);
  EXPECT_LE(sent, 105u);                            // ~100, not 400
}

TEST(HardwareInterface, DeactivationFlushesTheFirmwareQueueWithStop)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
  ASSERT_TRUE(r.go(lc::State::PRIMARY_STATE_INACTIVE, "inactive"));   // operator, no fault
  for (int i = 0; i < 100 && r.fake.count("ServoMoveEnd") == 0; ++i) {std::this_thread::sleep_for(10ms);}
  EXPECT_EQ(r.fake.stops(), 1);                     // STOP first (sdk_quirks S15/S16)
  EXPECT_EQ(r.fake.count("ServoMoveEnd"), 1);
  ASSERT_TRUE(r.activate());                        // and it can be started again
  EXPECT_EQ(r.fake.count("ServoMoveStart"), 2);
}

// The status timer and the services run on other threads than the lifecycle callbacks; nothing they
// use may be replaced underneath them (run under ThreadSanitizer to see a race, not only a crash).
TEST(HardwareInterface, ReconfiguringWhileTheStatusTimerRunsIsSafe)
{
  Rig r("", 125, true);
  for (int i = 0; i < 15; ++i) {
    ASSERT_TRUE(r.configure()) << i;
    r.run(2);
    if (i % 3 == 0) {
      ASSERT_TRUE(r.activate()) << i;
      r.run(2);
      ASSERT_TRUE(r.go(lc::State::PRIMARY_STATE_INACTIVE, "inactive")) << i;
    }
    ASSERT_TRUE(r.go(lc::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured")) << i;   // cleanup
    std::this_thread::sleep_for(30ms);
  }
  EXPECT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "hardware_state") == "unconfigured";}));
}

TEST(HardwareInterface, InconsistentRatesAreRejectedAtInit)
{
  // 100 Hz controller with cmdT 8 ms: 0.8 cycles per segment
  Rig r("", 100);
  EXPECT_EQ(r.rm->system_components_size(), 0u);    // the component did not load
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
