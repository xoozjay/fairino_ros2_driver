// State machine of FrRobotHardwareInterface against a fake controller, driven through the
// ResourceManager exactly as the controller manager does (docs/fault_handling.md).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include <unistd.h>

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
std::string xacro(const std::string & args)
{
  const std::string share = ament_index_cpp::get_package_share_directory("fr_robot_driver");
  return run("xacro " + share + "/urdf/fr_controlled.urdf.xacro model:=fairino5_v6 " + args +
           " 2>/dev/null");
}

// The plugin's node lives in a namespace of this process only: a real driver running on the same
// ROS graph has a node of the same name, and a test must never call its services.
const std::string kNs = "fr_test_" + std::to_string(::getpid());

std::string urdf(const fr_test::FakeController & c, const std::string & extra_params = "",
  int update_rate_hz = 125, const std::string & xacro_args = "")
{
  std::string xml = xacro("use_mock_hardware:=false ip:=127.0.0.1 namespace:=" + kNs + " update_rate_hz:=" +
      std::to_string(update_rate_hz) + " " + xacro_args);
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
    const std::string & xacro_args = "", bool multi_threaded = false)
  : update_rate(update_rate_hz)
  {
    if (multi_threaded) {
      executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    } else {
      executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf(fake, extra, update_rate_hz, xacro_args);
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    params.logger = rclcpp::get_logger("test");
    params.executor = executor;
    params.update_rate = update_rate_hz;
    rm = std::make_unique<hardware_interface::ResourceManager>(params, true);
    status_node = std::make_shared<rclcpp::Node>("status_listener");
    status_sub = status_node->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "/" + kNs + "/fr_robot_hw/status", rclcpp::QoS(1).transient_local(),
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
  std::pair<bool, std::string> reset() {return trigger("reset_error");}

  // Calls the Trigger service ~/<name> like an operator; returns (success, message).
  std::pair<bool, std::string> trigger(const std::string & name)
  {
    auto node = std::make_shared<rclcpp::Node>("operator");
    auto client = node->create_client<std_srvs::srv::Trigger>("/" + kNs + "/fr_robot_hw/" + name);
    if (!client->wait_for_service(2s)) {return {false, "service not available"};}
    auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    if (ex.spin_until_future_complete(fut, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
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
  Rig r("", 125, "gripper:=pgi140", true);
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

// ---- the gripper (sdk_quirks G1-G7) ----

namespace
{
const char * kWithGripper = "gripper:=pgi140";
const char * kFinger = "left_finger_joint/position";
const std::vector<std::string> kGripperCommands = {
  "left_finger_joint/position", "left_finger_joint/max_velocity", "left_finger_joint/max_effort"};

// The gripper controller's view: its three command interfaces, claimed.
struct GripperController
{
  explicit GripperController(Rig & rig)
  : r(rig),
    position(r.rm->claim_command_interface(kGripperCommands[0])),
    velocity(r.rm->claim_command_interface(kGripperCommands[1])),
    effort(r.rm->claim_command_interface(kGripperCommands[2])) {}
  // what the controller manager does when gripper_controller is activated / deactivated
  void start() {ASSERT_TRUE(r.rm->perform_command_mode_switch(kGripperCommands, {}));}
  void stop() {ASSERT_TRUE(r.rm->perform_command_mode_switch({}, kGripperCommands));}
  double command() {return position.get_optional<double>().value_or(std::nan(""));}
  // one controller cycle with this goal (velocity/effort 0: none given)
  void command(double opening, double max_velocity = 0.0, double max_effort = 0.0)
  {
    r.cycle([&] {
        (void)position.set_value(opening);
        (void)velocity.set_value(max_velocity);
        (void)effort.set_value(max_effort);
      });
  }
  Rig & r;
  hardware_interface::LoanedCommandInterface position, velocity, effort;
};

// Runs controller cycles until `pred` holds (the gripper sender works asynchronously).
bool cycleUntil(Rig & r, const std::function<bool()> & pred, int cycles = 125)
{
  for (int i = 0; i < cycles && !pred(); ++i) {r.cycle();}
  return pred();
}

const std::vector<double> kMove50 = {1, 50, 50, 20, 30000, 1, 0, 0, 0, 0};
}  // namespace

TEST(Gripper, MockDescriptionHasTheFingerJoint)
{
  hardware_interface::ResourceManagerParams params;
  params.robot_description = xacro(kWithGripper);   // mock hardware
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  params.logger = rclcpp::get_logger("test");
  hardware_interface::ResourceManager rm(params, true);
  for (const auto & c : kGripperCommands) {
    EXPECT_TRUE(rm.command_interface_exists(c)) << c;
  }
  EXPECT_TRUE(rm.state_interface_exists("left_finger_joint/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("speed_scaling/speed_scaling_factor"));
  EXPECT_FALSE(rm.state_interface_exists("right_finger_joint/position"));   // a mimic joint: TF only
  EXPECT_NE(params.robot_description.find("<parent link=\"tool0\"/>"), std::string::npos);
}

TEST(Gripper, CommandsAreSentOnlyWhenTheyChange)
{
  Rig r("", 125, kWithGripper);
  r.fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 30; s.gripper_motiondone = 1;});
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(3);
  EXPECT_NEAR(r.state(kFinger), 0.012, 1e-9);        // 30 % of the 0.04 m stroke

  GripperController g(r);
  g.start();
  EXPECT_NEAR(g.command(), 0.012, 1e-9);             // starts from the reported opening
  r.run(20);
  EXPECT_EQ(r.fake.count("MoveGripper"), 0);        // holding it: nothing to send

  g.command(0.02);                                   // no velocity, no effort given
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));
  EXPECT_EQ(r.fake.params("MoveGripper")[0], kMove50);  // 50 %, default 50 % speed, 20 % force
  for (int i = 0; i < 20; ++i) {g.command(0.02);}
  g.command(0.0201);                                 // within 1 % of the stroke
  r.run(20);
  EXPECT_EQ(r.fake.count("MoveGripper"), 1);
  EXPECT_NEAR(r.state(kFinger), 0.02, 1e-9);

  g.command(0.04, 0.011, 14.0);                      // 25 % speed, 10 % force
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 2;}));
  EXPECT_EQ(r.fake.params("MoveGripper")[1], (std::vector<double>{1, 100, 25, 10, 30000, 1, 0, 0, 0, 0}));
  g.command(0.08, 1.0, 1000.0);                      // beyond the stroke and the scales: clamped
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 2;}));
  g.command(-0.01);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 3;}));
  EXPECT_EQ(r.fake.params("MoveGripper")[2], (std::vector<double>{1, 0, 50, 20, 30000, 1, 0, 0, 0, 0}));
  EXPECT_EQ(r.fake.count("ActGripper"), 0);         // never activated by the driver
}

TEST(Gripper, NothingIsSentUnlessTheControllerStartedOnAUsableGripper)
{
  Rig r("", 125, kWithGripper);                      // just powered up: the gripper is not active
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(3);
  EXPECT_NEAR(r.state(kFinger), 0.04, 1e-9);         // unknown: the widest outline, not the frame's 0 %
  ASSERT_TRUE(r.waitStatus([](const auto & s) {
      return Rig::value(s, "gripper_position_source") == "unknown_not_active";}));
  GripperController g(r);
  g.start();                                         // not usable: not armed
  g.command(0.02);
  r.run(20);
  EXPECT_EQ(r.fake.count("MoveGripper"), 0);

  const auto [ok, message] = r.trigger("gripper/activate");   // operator
  ASSERT_TRUE(ok) << message;
  r.run(5);
  EXPECT_NEAR(r.state(kFinger), 0.04, 1e-9);         // open, though the frame says 0 %
  g.command(0.02);                                   // the goal from before the activation...
  r.run(30);
  EXPECT_EQ(r.fake.count("MoveGripper"), 0);        // ...is never sent

  g.stop();                                          // operator restarts gripper_controller
  g.start();
  EXPECT_NEAR(g.command(), 0.04, 1e-9);
  r.run(10);
  EXPECT_EQ(r.fake.count("MoveGripper"), 0);
  g.command(0.0);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));

  // a gripper fault disarms; the commands stay ignored after it is gone, until a restart
  r.fake.edit([](auto & s) {s.gripper_fault = 1;});
  r.run(3);
  g.command(0.02);
  r.run(20);
  r.fake.edit([](auto & s) {s.gripper_fault = 0;});
  r.run(3);
  g.command(0.03);
  r.run(20);
  EXPECT_EQ(r.fake.count("MoveGripper"), 1);

  // a robot fault: the hardware deactivates, nothing goes out
  g.stop();
  g.start();
  r.fake.edit([](auto & s) {s.main_code = 1; s.sub_code = 23;});
  std::this_thread::sleep_for(50ms);                 // the next frames carry it
  for (int i = 0; i < 30 && r.stateId() == lc::State::PRIMARY_STATE_ACTIVE; ++i) {
    g.command(0.01);
  }
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_INACTIVE);
  r.run(20);
  EXPECT_EQ(r.fake.count("MoveGripper"), 1);
}

TEST(Gripper, ActivationOnlyOnRequestAndOnlyOnce)
{
  Rig r("", 125, kWithGripper);
  ASSERT_TRUE(r.configure());
  r.run(3);
  ASSERT_TRUE(r.activate());
  r.run(3);
  EXPECT_EQ(r.fake.count("ActGripper"), 0);         // neither configure nor activate does it
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "gripper_active") == "0";}));

  r.fake.edit([](auto & s) {s.actual_qd[2] = 5.0;});
  std::this_thread::sleep_for(50ms);
  auto [ok, message] = r.trigger("gripper/activate");
  EXPECT_FALSE(ok);
  EXPECT_NE(message.find("moving"), std::string::npos) << message;
  r.fake.edit([](auto & s) {s.actual_qd[2] = 0.0; s.main_code = 1; s.sub_code = 23;});
  std::this_thread::sleep_for(50ms);                 // the next frames carry it
  std::tie(ok, message) = r.trigger("gripper/activate");
  EXPECT_FALSE(ok);
  EXPECT_NE(message.find("fault"), std::string::npos) << message;
  EXPECT_EQ(r.fake.count("ActGripper"), 0);
  ASSERT_TRUE(r.reset().first);                      // the operator clears the robot fault...
  r.run(5);
  ASSERT_TRUE(r.activate());                         // ...and reactivates
  r.run(10);
  r.fake.edit([](auto & s) {s.gripper_fault = 1;});  // a faulted, inactive gripper (G11)
  std::this_thread::sleep_for(50ms);

  std::tie(ok, message) = r.trigger("gripper/activate");   // activating is what clears it
  ASSERT_TRUE(ok) << message;
  EXPECT_EQ(r.fake.params("ActGripper"), (std::vector<std::vector<double>>{{1, 1}}));
  EXPECT_EQ(r.fake.state().gripper_fault, 0);
  r.run(3);
  EXPECT_NEAR(r.state(kFinger), 0.04, 1e-9);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {
      return Rig::value(s, "gripper_active") == "1" &&
      Rig::value(s, "gripper_position_source") == "inferred_open_after_activation";}));
  EXPECT_EQ(Rig::value(r.status(), "gripper_commands"), "not armed");

  std::tie(ok, message) = r.trigger("gripper/activate");
  EXPECT_FALSE(ok);
  EXPECT_NE(message.find("already active"), std::string::npos) << message;
  EXPECT_EQ(r.fake.count("ActGripper"), 1);
}

TEST(Gripper, TheOpeningIsHeldWhileTheFrameSaysZero)
{
  Rig r("", 125, kWithGripper);
  // activated before the driver started and never moved since (sdk_quirks G5)
  r.fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 0; s.gripper_motiondone = 0;});
  r.fake.gripper_motion_ms = 400;                    // the frame says "0 %, not done" meanwhile (G10)
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
  EXPECT_NEAR(r.state(kFinger), 0.04, 1e-9);         // fully open, though the frame says 0 %
  GripperController g(r);
  g.start();
  EXPECT_NEAR(g.command(), 0.04, 1e-9);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "gripper_commands") == "armed";}));

  g.command(0.02);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));
  r.run(10);
  EXPECT_NEAR(r.state(kFinger), 0.04, 1e-9);         // still where it was known to be
  ASSERT_TRUE(cycleUntil(r, [&] {return std::abs(r.state(kFinger) - 0.02) < 1e-9;}));
  g.command(0.0);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 2;}));
  r.run(10);
  EXPECT_NEAR(r.state(kFinger), 0.02, 1e-9);         // held during the motion...
  ASSERT_TRUE(r.waitStatus([](const auto & s) {
      return Rig::value(s, "gripper_position_source") == "held_during_motion";}));
  ASSERT_TRUE(cycleUntil(r, [&] {return r.state(kFinger) == 0.0;}));   // ...closed and done: 0 %
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "gripper_position_source") == "frame";}));
}

TEST(Gripper, WriteNeverWaitsAndOnlyTheNewestCommandGoesOut)
{
  Rig r("", 125, kWithGripper);
  r.fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 100; s.gripper_motiondone = 1;});
  r.fake.move_gripper_delay_ms = 300;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(20);                                         // the stream pauses
  GripperController g(r);
  g.start();

  const auto timed = [&](double opening) {
      const auto t0 = std::chrono::steady_clock::now();
      g.command(opening);
      return std::chrono::steady_clock::now() - t0;
    };
  EXPECT_LT(timed(0.01), 50ms);                      // a cycle includes an 8 ms sleep
  std::this_thread::sleep_for(50ms);                 // 25 % is in flight for 300 ms
  EXPECT_LT(timed(0.02), 50ms);
  EXPECT_LT(timed(0.03), 50ms);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 2;}));
  r.run(60);
  const auto sent = r.fake.params("MoveGripper");
  ASSERT_EQ(sent.size(), 2u);
  EXPECT_EQ(sent[0][1], 25);
  EXPECT_EQ(sent[1][1], 75);                         // 50 % was overtaken before it went out
}

TEST(HardwareInterface, TheStreamPausesWhileTheArmStandsAndResumesWithAPrefill)
{
  Rig r;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  auto cmd = r.rm->claim_command_interface(kPan);
  r.run(20);
  const auto standing = r.fake.servoJ().size();     // the prefill and a repeat, then nothing
  r.run(50);
  EXPECT_EQ(r.fake.servoJ().size(), standing);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "servo_stream") == "paused";}));

  for (int i = 1; i <= 50; ++i) {                   // a ramp: streams again
    r.cycle([&] {(void)cmd.set_value(i * 0.5 * kDeg);});
  }
  const auto sent = r.fake.servoJ();
  ASSERT_GT(sent.size(), standing + 40);
  for (std::size_t i = standing; i < standing + 3; ++i) {
    EXPECT_NEAR(sent[i][0], 0.0, 1e-9) << i;         // the prefill stands where the arm stood
  }
  ASSERT_TRUE(cycleUntil(r, [&] {return std::abs(r.fake.servoJ().back()[0] - 25.0) < 1e-6;}));
  r.run(20);
  const auto stopped = r.fake.servoJ().size();
  r.run(30);
  EXPECT_EQ(r.fake.servoJ().size(), stopped);        // standing again: paused
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);
}

TEST(Gripper, TrajectoriesPauseWhileTheGripperIsBusy)
{
  const char * kScaling = "speed_scaling/speed_scaling_factor";
  {
    Rig plain;                                       // no gripper: never paused
    ASSERT_TRUE(plain.configure());
    ASSERT_TRUE(plain.activate());
    plain.run(3);
    EXPECT_EQ(plain.state(kScaling), 1.0);
  }
  Rig r("", 125, kWithGripper);
  r.fake.gripper_activation_ms = 300;
  r.fake.gripper_motion_ms = 300;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(3);
  EXPECT_EQ(r.state(kScaling), 1.0);

  // the operator's activation moves the gripper too (it opens fully)
  auto activation = std::async(std::launch::async, [&] {return r.trigger("gripper/activate");});
  EXPECT_TRUE(cycleUntil(r, [&] {return r.state(kScaling) == 0.0;}));
  while (activation.wait_for(0s) != std::future_status::ready) {r.cycle();}
  ASSERT_TRUE(activation.get().first);
  r.cycle();
  EXPECT_EQ(r.state(kScaling), 1.0);

  GripperController g(r);
  g.start();
  g.command(0.02);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));
  r.cycle();
  EXPECT_EQ(r.state(kScaling), 0.0);                 // from the MoveGripper...
  ASSERT_TRUE(cycleUntil(r, [&] {return r.state(kScaling) == 1.0;}));
  EXPECT_NEAR(r.state(kFinger), 0.02, 1e-9);         // ...until the frame shows it done
}

TEST(Gripper, ACommandWaitsForTheArmToStand)
{
  Rig r("", 125, kWithGripper);
  r.fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 100; s.gripper_motiondone = 1;});
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
  GripperController g(r);
  g.start();
  auto pan = r.rm->claim_command_interface(kPan);
  for (int i = 1; i <= 50; ++i) {                    // the arm moves; the gripper is asked to close
    r.cycle([&] {
        (void)pan.set_value(i * 0.5 * kDeg);
        (void)g.position.set_value(0.0);
      });
  }
  EXPECT_EQ(r.fake.count("MoveGripper"), 0);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));   // once it stands
  EXPECT_EQ(r.fake.move_gripper_while_streaming, 0);
  EXPECT_EQ(r.fake.params("MoveGripper")[0][1], 0);
}

TEST(Gripper, AnArmCommandWaitsWhileTheGripperMovesAndGoesOnRightAfter)
{
  Rig r("", 125, kWithGripper);
  r.fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 100; s.gripper_motiondone = 1;});
  r.fake.gripper_motion_ms = 600;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
  GripperController g(r);
  g.start();
  auto pan = r.rm->claim_command_interface(kPan);
  g.command(0.02);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));
  const auto before = r.fake.servoJ().size();
  // A trajectory starts while the gripper moves. The paused trajectory controller holds its first
  // step: 0.24 deg, one 8 ms cycle at 30 deg/s.
  for (int i = 0; i < 20; ++i) {
    r.cycle([&] {(void)pan.set_value(0.24 * kDeg);});
  }
  EXPECT_EQ(r.fake.servoJ().size(), before);         // the arm stays: nothing went out
  ASSERT_TRUE(r.waitStatus([](const auto & s) {
      return Rig::value(s, "arm_commands") == "paused (gripper busy)";}));
  EXPECT_EQ(r.state("speed_scaling/speed_scaling_factor"), 0.0);
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);

  // the gripper is done: the trajectory goes on from where it waited, at once
  ASSERT_TRUE(cycleUntil(r, [&] {return r.state("speed_scaling/speed_scaling_factor") == 1.0;}, 200));
  for (int i = 2; i <= 21; ++i) {
    r.cycle([&] {(void)pan.set_value(i * 0.24 * kDeg);});
  }
  ASSERT_TRUE(cycleUntil(r, [&] {return std::abs(r.fake.servoJ().back()[0] - 21 * 0.24) < 1e-6;}));
  EXPECT_GT(r.fake.servoJ().size(), before + 15);
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(r.state("fr_status/fault_class"), 0.0);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "arm_commands") == "accepted";}));
  EXPECT_EQ(r.fake.servoj_while_gripper_moves, 0);
  EXPECT_EQ(r.fake.move_gripper_while_streaming, 0);
}

TEST(Gripper, AControllerThatIgnoresThePauseEndsInTheStepCheck)
{
  Rig r("", 125, kWithGripper);
  r.fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 100; s.gripper_motiondone = 1;});
  r.fake.gripper_motion_ms = 300;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
  GripperController g(r);
  g.start();
  auto pan = r.rm->claim_command_interface(kPan);
  g.command(0.02);
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));
  const auto before = r.fake.servoJ().size();
  for (int i = 1; i <= 20; ++i) {                    // runs on while the gripper moves: 10 deg away
    r.cycle([&] {(void)pan.set_value(i * 0.5 * kDeg);});
  }
  bool deactivated = false;
  for (int i = 0; i < 200 && !deactivated; ++i) {
    deactivated = r.cycle().result == return_type::DEACTIVATE;
  }
  ASSERT_TRUE(deactivated);                          // the jump is refused once the gripper is done
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(r.fake.servoJ().size(), before);
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "fault_class") == "1";}));
  EXPECT_NE(r.status().message.find("step check"), std::string::npos) << r.status().message;
  EXPECT_EQ(r.fake.servoj_while_gripper_moves, 0);
}

TEST(Gripper, AFailedCallKeepsTheArmStillUntilTheFrameShowsTheGripperDone)
{
  Rig r("", 125, kWithGripper);
  r.fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 100; s.gripper_motiondone = 1;});
  r.fake.move_gripper_delay_ms = 1200;   // past the call's 1 s timeout: it fails, the gripper moves anyway
  r.fake.gripper_motion_ms = 600;
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
  GripperController g(r);
  g.start();
  auto pan = r.rm->claim_command_interface(kPan);
  g.command(0.02);
  ASSERT_TRUE(cycleUntil(r, [&] {
      return Rig::value(r.status(), "gripper_command_failures") == "1";}, 250));
  EXPECT_EQ(Rig::value(r.status(), "gripper_busy"), "1");
  const auto before = r.fake.servoJ().size();
  for (int i = 1; i <= 20; ++i) {                    // a trajectory starts after the failure
    r.cycle([&] {(void)pan.set_value(i * 0.05 * kDeg);});
  }
  EXPECT_EQ(r.fake.servoJ().size(), before);         // still refused: the gripper may be moving
  ASSERT_TRUE(cycleUntil(r, [&] {return Rig::value(r.status(), "gripper_busy") == "0";}, 250));
  EXPECT_EQ(r.fake.count("MoveGripper"), 1);
  EXPECT_NEAR(r.state(kFinger), 0.02, 1e-9);
  EXPECT_EQ(r.fake.servoj_while_gripper_moves, 0);

  r.cycle([&] {(void)pan.set_value(0.0);});          // within a step of where the arm stands: accepted
  for (int i = 1; i <= 20; ++i) {
    r.cycle([&] {(void)pan.set_value(i * 0.5 * kDeg);});
  }
  EXPECT_GT(r.fake.servoJ().size(), before + 15);
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
