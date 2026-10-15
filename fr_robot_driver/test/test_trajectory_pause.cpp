// joint_trajectory_controller (with the installed controllers.yaml) and the plugin together under a
// controller manager loop, against the fake controller: a trajectory that meets a gripper motion
// pauses and resumes instead of failing (speed_scaling_factor, sdk_quirks G9).
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "controller_manager/controller_manager.hpp"
#include "fake_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kDeg = M_PI / 180.0;
const std::string kNs = "fr_pause_" + std::to_string(::getpid());
const std::string kShare = ament_index_cpp::get_package_share_directory("fr_robot_driver");

std::string run(const std::string & cmd)
{
  std::string out;
  FILE * p = ::popen(cmd.c_str(), "r");
  char buf[4096];
  while (p && std::fgets(buf, sizeof(buf), p)) {out += buf;}
  if (p) {::pclose(p);}
  return out;
}

std::string replaceAll(std::string s, const std::string & from, const std::string & to)
{
  for (auto at = s.find(from); at != std::string::npos; at = s.find(from, at + to.size())) {
    s.replace(at, from.size(), to);
  }
  return s;
}

// The launch file's description and controller configuration, pointed at the fake controller.
std::string urdf(const fr_test::FakeController & c)
{
  std::string xml = run("xacro " + kShare + "/urdf/fr_controlled.urdf.xacro model:=fairino5_v6 "
      "gripper:=pgi140 use_mock_hardware:=false ip:=127.0.0.1 update_rate_hz:=125 namespace:=" + kNs +
      " 2>/dev/null");
  const std::string ports = "<param name=\"xmlrpc_port\">" + std::to_string(c.xmlrpcPort()) +
    "</param><param name=\"state_port\">" + std::to_string(c.statePort()) +
    "</param><param name=\"instant_port\">" + std::to_string(c.instantPort()) + "</param>";
  const auto at = xml.find("</hardware>");
  return xml.substr(0, at) + ports + xml.substr(at);
}

std::string controllersFile()
{
  std::ifstream in(kShare + "/config/controllers.yaml");
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const std::string path = "/tmp/" + kNs + "_controllers.yaml";
  std::ofstream(path) << replaceAll(replaceAll(text, "__PREFIX__", ""), "__NS__", "/" + kNs);
  return path;
}

using Trajectory = control_msgs::action::FollowJointTrajectory;
using Gripper = control_msgs::action::ParallelGripperCommand;
template<typename A>
using Result = typename rclcpp_action::ClientGoalHandle<A>::WrappedResult;

// A controller manager as ros2_control_node runs it: its update loop at 125 Hz on one thread, the
// executor on another; the hardware active, joint_trajectory_controller active.
struct CmRig
{
  CmRig()
  {
    fake.edit([](auto & s) {s.gripper_active = 1; s.gripper_position = 100; s.gripper_motiondone = 1;});
    auto options = controller_manager::get_cm_node_options();
    options.parameter_overrides({
        {"update_rate", 125},
        {"joint_trajectory_controller.type", "joint_trajectory_controller/JointTrajectoryController"},
        {"joint_trajectory_controller.params_file", controllersFile()}});
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    cm = std::make_shared<controller_manager::ControllerManager>(
      executor, urdf(fake), true, "controller_manager", kNs, options);
    executor->add_node(cm);
    client_node = std::make_shared<rclcpp::Node>("pause_client", kNs);
    trajectory = rclcpp_action::create_client<Trajectory>(
      client_node, "joint_trajectory_controller/follow_joint_trajectory");
    gripper = rclcpp_action::create_client<Gripper>(client_node, "gripper_controller/gripper_cmd");
    executor->add_node(client_node);
    spinner = std::thread([this] {executor->spin();});
    loop = std::thread([this] {
        const auto period = rclcpp::Duration::from_seconds(1.0 / 125);
        auto next = std::chrono::steady_clock::now();
        while (running) {
          const auto now = cm->get_trigger_clock()->now();
          cm->read(now, period);
          cm->update(now, period);
          cm->write(now, period);
          next += 8ms;
          std::this_thread::sleep_until(next);
        }
      });
    ok = cm->load_controller("joint_trajectory_controller") != nullptr &&
      cm->configure_controller("joint_trajectory_controller") == controller_interface::return_type::OK &&
      cm->switch_controller({"joint_trajectory_controller"}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT) ==
      controller_interface::return_type::OK &&
      trajectory->wait_for_action_server(5s) && gripper->wait_for_action_server(5s);
  }
  ~CmRig()
  {
    running = false;
    loop.join();
    while (!executor->is_spinning()) {std::this_thread::yield();}
    executor->cancel();
    spinner.join();
    executor->remove_node(client_node);
    executor->remove_node(cm);
    cm.reset();
  }

  // shoulder_pan_joint from where it is to `deg` in `seconds`, one point: linear interpolation
  rclcpp_action::ClientGoalHandle<Trajectory>::SharedPtr move(double deg, double seconds)
  {
    Trajectory::Goal goal;
    goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    const auto now = fake.state();                   // the other joints stay where they are
    for (int i = 0; i < 6; ++i) {p.positions.push_back(now.jt_cur_pos[i] * kDeg);}
    p.positions[0] = deg * kDeg;
    p.time_from_start = rclcpp::Duration::from_seconds(seconds);
    goal.trajectory.points = {p};
    auto f = trajectory->async_send_goal(goal);
    return f.wait_for(2s) == std::future_status::ready ? f.get() : nullptr;
  }
  rclcpp_action::ClientGoalHandle<Gripper>::SharedPtr grip(double opening)
  {
    Gripper::Goal goal;
    goal.command.name = {"left_finger_joint"};
    goal.command.position = {opening};
    auto f = gripper->async_send_goal(goal);
    return f.wait_for(2s) == std::future_status::ready ? f.get() : nullptr;
  }
  template<typename A>
  std::optional<Result<A>> result(
    const std::shared_ptr<rclcpp_action::Client<A>> & client,
    const typename rclcpp_action::ClientGoalHandle<A>::SharedPtr & h, std::chrono::milliseconds timeout = 5s)
  {
    auto f = client->async_get_result(h);
    if (f.wait_for(timeout) != std::future_status::ready) {return std::nullopt;}
    return f.get();
  }
  bool until(const std::function<bool()> & pred, std::chrono::milliseconds timeout = 3s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!pred() && std::chrono::steady_clock::now() < deadline) {std::this_thread::sleep_for(5ms);}
    return pred();
  }
  double pan() {return fake.state().jt_cur_pos[0];}   // deg, the last ServoJ

  fr_test::FakeController fake;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> cm;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_action::Client<Trajectory>::SharedPtr trajectory;
  rclcpp_action::Client<Gripper>::SharedPtr gripper;
  std::atomic<bool> running{true};
  std::thread spinner, loop;
  bool ok = false;
};
}  // namespace

// A slow trajectory, and a fast one (30 deg/s from the first instant) whose first step is ahead of
// the arm while it waits: both pause until the gripper is done, then run and succeed.
TEST(TrajectoryPause, ATrajectoryThatMeetsAGripperMotionWaitsAndSucceeds)
{
  CmRig r;
  ASSERT_TRUE(r.ok);
  r.fake.gripper_motion_ms = 600;
  for (const auto & [deg, seconds] : {std::pair{2.0, 0.5}, std::pair{17.0, 0.5}}) {
    const int sent = r.fake.count("MoveGripper");
    auto g = r.grip(r.fake.state().gripper_position == 100 ? 0.0 : 0.04);
    ASSERT_TRUE(g);
    ASSERT_TRUE(r.until([&] {return r.fake.state().gripper_motiondone == 0;}));   // it moves
    const double start = r.pan();
    auto t = r.move(deg, seconds);
    ASSERT_TRUE(t);
    std::this_thread::sleep_for(300ms);
    EXPECT_NEAR(r.pan(), start, 1e-9) << deg;        // paused: the arm has not moved
    auto tr = r.result(r.trajectory, t);
    ASSERT_TRUE(tr) << deg;
    EXPECT_EQ(tr->code, rclcpp_action::ResultCode::SUCCEEDED) << deg << ": " << tr->result->error_string;
    EXPECT_NEAR(r.pan(), deg, 0.29);                 // the goal tolerance, 0.005 rad
    auto gr = r.result(r.gripper, g);
    ASSERT_TRUE(gr);
    EXPECT_EQ(gr->code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(r.fake.count("MoveGripper"), sent + 1);
  }
  EXPECT_EQ(r.fake.servoj_while_gripper_moves, 0);
  EXPECT_EQ(r.fake.count("ResetAllError"), 0);
}

TEST(TrajectoryPause, AGripperGoalWaitsForARunningTrajectory)
{
  CmRig r;
  ASSERT_TRUE(r.ok);
  auto t = r.move(10.0, 1.0);
  ASSERT_TRUE(t);
  ASSERT_TRUE(r.until([&] {return r.pan() > 1.0;}));
  auto g = r.grip(0.02);
  ASSERT_TRUE(g);
  auto tr = r.result(r.trajectory, t);
  ASSERT_TRUE(tr);
  EXPECT_EQ(tr->code, rclcpp_action::ResultCode::SUCCEEDED);
  auto gr = r.result(r.gripper, g);
  ASSERT_TRUE(gr);
  EXPECT_EQ(gr->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(r.fake.count("MoveGripper"), 1);
  EXPECT_EQ(r.fake.move_gripper_while_streaming, 0);
}

TEST(TrajectoryPause, APausedTrajectoryCanBeCanceled)
{
  CmRig r;
  ASSERT_TRUE(r.ok);
  r.fake.gripper_motion_ms = 600;
  auto g = r.grip(0.0);
  ASSERT_TRUE(g);
  ASSERT_TRUE(r.until([&] {return r.fake.state().gripper_motiondone == 0;}));
  auto t = r.move(10.0, 0.5);
  ASSERT_TRUE(t);
  std::this_thread::sleep_for(100ms);
  ASSERT_EQ(r.trajectory->async_cancel_goal(t).wait_for(2s), std::future_status::ready);
  auto tr = r.result(r.trajectory, t);
  ASSERT_TRUE(tr);
  EXPECT_EQ(tr->code, rclcpp_action::ResultCode::CANCELED);
  auto gr = r.result(r.gripper, g);
  ASSERT_TRUE(gr);
  std::this_thread::sleep_for(300ms);                // after the gripper: the arm stays
  EXPECT_NEAR(r.pan(), 0.0, 0.01);
  EXPECT_EQ(r.fake.count("ResetAllError"), 0);
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
