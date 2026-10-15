// State machine of FrRobotHardwareInterface against a fake controller, driven through the
// ResourceManager exactly as the controller manager does (docs/fault_handling.md).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "fake_controller.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
  "left_finger_joint/position", "left_finger_joint/max_effort"};
const char * kScaling = "speed_scaling/speed_scaling_factor";

// Runs controller cycles until `pred` holds (the gripper sender works asynchronously).
bool cycleUntil(Rig & r, const std::function<bool()> & pred, int cycles = 125)
{
  for (int i = 0; i < cycles && !pred(); ++i) {r.cycle();}
  return pred();
}

// The plugin's gripper action, used like an upper layer (MoveIt, a script) does.
struct GripperClient
{
  using Action = control_msgs::action::ParallelGripperCommand;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  using Code = rclcpp_action::ResultCode;

  explicit GripperClient(Rig & rig)
  : r(rig), node(std::make_shared<rclcpp::Node>("gripper_client")),
    client(rclcpp_action::create_client<Action>(node, "/" + kNs + "/gripper_controller/gripper_cmd"))
  {
    r.executor->add_node(node);
  }
  ~GripperClient() {r.executor->remove_node(node);}

  // The goal's handle; null if rejected. velocity / effort < 0: not given.
  Handle::SharedPtr send(double opening, double velocity = -1.0, double effort = -1.0)
  {
    if (!client->wait_for_action_server(2s)) {return nullptr;}
    Action::Goal goal;
    goal.command.name = {"left_finger_joint"};
    goal.command.position = {opening};
    if (velocity >= 0.0) {goal.command.velocity = {velocity};}
    if (effort >= 0.0) {goal.command.effort = {effort};}
    auto f = client->async_send_goal(goal);
    return f.wait_for(2s) == std::future_status::ready ? f.get() : nullptr;
  }
  // Runs `step` (default: one controller cycle) until the goal ends; none if it does not.
  std::optional<Handle::WrappedResult> result(
    const Handle::SharedPtr & h, int cycles = 250, const std::function<void()> & step = {})
  {
    auto f = client->async_get_result(h);
    for (int i = 0; i < cycles && f.wait_for(0s) != std::future_status::ready; ++i) {
      step ? step() : (void)r.cycle();
    }
    if (f.wait_for(0s) != std::future_status::ready) {return std::nullopt;}
    return f.get();
  }
  bool cancel(const Handle::SharedPtr & h)
  {
    return client->async_cancel_goal(h).wait_for(2s) == std::future_status::ready;
  }

  Rig & r;
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<Action>::SharedPtr client;
};

// A rig with an active, usable gripper at `percent`.
void startWithGripperAt(Rig & r, int percent)
{
  r.fake.edit([&](auto & s) {s.gripper_active = 1; s.gripper_position = percent; s.gripper_motiondone = 1;});
  ASSERT_TRUE(r.configure());
  ASSERT_TRUE(r.activate());
  r.run(10);
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
  EXPECT_FALSE(rm.command_interface_exists("left_finger_joint/max_velocity"));
  EXPECT_TRUE(rm.state_interface_exists("left_finger_joint/velocity"));
  EXPECT_TRUE(rm.state_interface_exists(kScaling));
  EXPECT_FALSE(rm.state_interface_exists("right_finger_joint/position"));   // a mimic joint: TF only
  EXPECT_NE(params.robot_description.find("<parent link=\"tool0\"/>"), std::string::npos);
}

TEST(Gripper, GoalsSendPositionAndForceAndIgnoreVelocity)
{
  Rig r("", 125, kWithGripper);
  startWithGripperAt(r, 30);
  EXPECT_NEAR(r.state(kFinger), 0.012, 1e-9);        // 30 % of the 0.04 m stroke
  GripperClient c(r);

  auto h = c.send(0.02);                             // no velocity, no effort given
  ASSERT_TRUE(h);
  auto res = c.result(h);
  ASSERT_TRUE(res);
  EXPECT_EQ(res->code, GripperClient::Code::SUCCEEDED);
  EXPECT_TRUE(res->result->reached_goal);
  EXPECT_FALSE(res->result->stalled);
  EXPECT_NEAR(res->result->state.position.at(0), 0.02, 1e-9);
  EXPECT_DOUBLE_EQ(r.state("left_finger_joint/velocity"), 0.0);
  EXPECT_EQ(r.fake.params("MoveGripper").at(0), kMove50);  // 50 % opening, fixed 50 % speed, 20 % force

  h = c.send(0.02, 0.011, 14.0);                    // a velocity is given; it is ignored (G12)
  ASSERT_TRUE(h);
  ASSERT_TRUE(c.result(h));
  // opening %, the fixed protocol speed 50, the force from 14 N, the SDK's max_time and flags
  EXPECT_EQ(r.fake.params("MoveGripper").back(), (std::vector<double>{1, 50, 50, 10, 30000, 1, 0, 0, 0, 0}));
  EXPECT_EQ(r.fake.count("ActGripper"), 0);         // never activated by the driver
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "gripper_motion_done") == "1";}));
  EXPECT_EQ(Rig::value(r.status(), "gripper_commands"), "<missing>");
  EXPECT_EQ(Rig::value(r.status(), "gripper_object"), "<missing>");   // not observable (G13)
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
  ASSERT_TRUE(r.waitStatus([](const auto & s) {return Rig::value(s, "gripper_active") == "1";}));

  std::tie(ok, message) = r.trigger("gripper/activate");
  EXPECT_FALSE(ok);
  EXPECT_NE(message.find("already active"), std::string::npos) << message;
  EXPECT_EQ(r.fake.count("ActGripper"), 1);
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

  GripperClient c(r);
  ASSERT_TRUE(c.send(0.02));
  ASSERT_TRUE(cycleUntil(r, [&] {return r.fake.count("MoveGripper") == 1;}));
  r.cycle();
  EXPECT_EQ(r.state(kScaling), 0.0);                 // from the MoveGripper...
  ASSERT_TRUE(cycleUntil(r, [&] {return r.state(kScaling) == 1.0;}));
  EXPECT_NEAR(r.state(kFinger), 0.02, 1e-9);         // ...until the frame shows it done
}

TEST(Gripper, AnArmCommandWaitsWhileTheGripperMovesAndGoesOnRightAfter)
{
  Rig r("", 125, kWithGripper);
  r.fake.gripper_motion_ms = 600;
  startWithGripperAt(r, 100);
  GripperClient c(r);
  auto pan = r.rm->claim_command_interface(kPan);
  ASSERT_TRUE(c.send(0.02));
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
  EXPECT_EQ(r.state(kScaling), 0.0);
  EXPECT_EQ(r.stateId(), lc::State::PRIMARY_STATE_ACTIVE);

  // the gripper is done: the trajectory goes on from where it waited, at once
  ASSERT_TRUE(cycleUntil(r, [&] {return r.state(kScaling) == 1.0;}, 200));
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
  r.fake.gripper_motion_ms = 300;
  startWithGripperAt(r, 100);
  GripperClient c(r);
  auto pan = r.rm->claim_command_interface(kPan);
  ASSERT_TRUE(c.send(0.02));
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
