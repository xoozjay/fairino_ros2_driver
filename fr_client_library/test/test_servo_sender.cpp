#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <thread>

#include "fr_client_library/servo_sender.hpp"
#include "fr_client_library/state_receiver.hpp"
#include "test_support.hpp"

using namespace fr_client_library;
using namespace fr_test;
using namespace std::chrono_literals;

namespace
{
// A controller that answers every XML-RPC call and records method names and bodies.
class Controller
{
public:
  Controller()
  : server_([this](int fd) {
        for (;;) {
          const std::string body = readHttpRequest(fd);
          if (body.empty()) {return;}
          const auto b = body.find("<methodName>") + 12;
          const std::string method = body.substr(b, body.find("</methodName>") - b);
          int answer = 0;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            methods_.push_back(method);
            bodies_.push_back(body);
            if (method == "ServoJ") {
              ++servo_j_;
              advance();
              level_ += 1.0;
              answer = servo_j_answer_;
              if (close_after_ > 0 && servo_j_ >= close_after_) {return;}
            } else if (method == "ServoMoveStart") {
              answer = start_answer_;
            }
          }
          if (delay_ms_ > 0 && method == "ServoJ") {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_.load()));
          }
          writeAll(fd, httpResponse(xmlResponse("<value><i4>" + std::to_string(answer) + "</i4></value>")));
        }
      }) {}

  uint16_t port() const {return server_.port();}
  std::vector<std::string> methods() const {std::lock_guard<std::mutex> l(mutex_); return methods_;}
  std::string lastServoJ() const
  {
    std::lock_guard<std::mutex> l(mutex_);
    for (auto i = bodies_.rbegin(); i != bodies_.rend(); ++i) {
      if (i->find("<methodName>ServoJ<") != std::string::npos) {return *i;}
    }
    return {};
  }
  // first joint of every ServoJ received, in order
  std::vector<double> servoJFirstJoints() const
  {
    std::lock_guard<std::mutex> l(mutex_);
    std::vector<double> out;
    for (const auto & b : bodies_) {
      if (b.find("<methodName>ServoJ<") == std::string::npos) {continue;}
      const auto p = b.find("<data><value><double>") + 21;
      out.push_back(std::stod(b.substr(p, b.find("</double>", p) - p)));
    }
    return out;
  }
  int servoJCount() const {std::lock_guard<std::mutex> l(mutex_); return servo_j_;}

  void setServoJAnswer(int v) {std::lock_guard<std::mutex> l(mutex_); servo_j_answer_ = v;}
  void setStartAnswer(int v) {std::lock_guard<std::mutex> l(mutex_); start_answer_ = v;}
  void closeAfterServoJ(int n) {std::lock_guard<std::mutex> l(mutex_); close_after_ = n;}
  void delayServoJ(int ms) {delay_ms_ = ms;}

  // The firmware queue: every ServoJ adds a segment, which the controller's own clock plays in
  // cmd_t_ms * (1 + drift_ppm * 1e-6) of host time. Positive drift: the controller is the slower.
  void playSegments(double cmd_t_ms, double drift_ppm)
  {
    std::lock_guard<std::mutex> l(mutex_);
    segment_s_ = cmd_t_ms * 1e-3 * (1.0 + drift_ppm * 1e-6);
    last_ = std::chrono::steady_clock::now();
  }
  int depth()
  {
    std::lock_guard<std::mutex> l(mutex_);
    advance();
    return static_cast<int>(std::ceil(level_));
  }

private:
  void advance()  // mutex_ held
  {
    const auto now = std::chrono::steady_clock::now();
    if (segment_s_ > 0) {
      level_ = std::max(0.0, level_ - std::chrono::duration<double>(now - last_).count() / segment_s_);
    }
    last_ = now;
  }

  mutable std::mutex mutex_;
  double segment_s_ = 0;
  double level_ = 0;
  std::chrono::steady_clock::time_point last_{};
  std::vector<std::string> methods_;
  std::vector<std::string> bodies_;
  int servo_j_ = 0;
  int servo_j_answer_ = 0;
  int start_answer_ = 0;
  int close_after_ = 0;
  std::atomic<int> delay_ms_{0};
  FakeServer server_;  // last: its thread uses the members above
};

ServoSenderOptions opts(int cmd_t_ms, int prefill = 0)
{
  ServoSenderOptions o;
  o.cmd_t_ms = cmd_t_ms;
  o.prefill = prefill;
  o.http = HttpOptions{500, 300};
  return o;
}

JointArray joint1(double v) {return {v, 0, 0, 0, 0, 0};}

template<typename Pred>
bool waitUntil(Pred p, int timeout_ms = 2000)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!p()) {
    if (std::chrono::steady_clock::now() > end) {return false;}
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

}  // namespace

TEST(ServoSender, StartStreamStopInOrder)
{
  Controller c;
  ServoSender s("127.0.0.1", c.port(), opts(5, 2));
  ASSERT_TRUE(s.start({1.5, 2, 3, 4, 5, 6}).ok());
  for (const double v : {9.0, 10.0, 11.0}) {ASSERT_TRUE(s.enqueue(joint1(v)));}
  ASSERT_TRUE(waitUntil([&] {return c.servoJCount() >= 5;}));
  EXPECT_TRUE(s.running());
  EXPECT_TRUE(s.stop().ok());
  EXPECT_FALSE(s.running());

  const auto m = c.methods();
  ASSERT_EQ(m.size(), 7u);
  EXPECT_EQ(m.front(), "ServoMoveStart");
  EXPECT_EQ(m[1], "ServoJ");
  EXPECT_EQ(m.back(), "ServoMoveEnd");
  EXPECT_NE(c.lastServoJ().find("<param><value><double>0.005000</double></value></param>"),
    std::string::npos);                                   // cmdT = configured period
  EXPECT_EQ(c.servoJFirstJoints(), (std::vector<double>{1.5, 1.5, 9, 10, 11}));
  EXPECT_EQ(s.stats().sent, 5u);
}

TEST(ServoSender, PrefillStandsStillAtTheInitialPositionBeforeAnyCommand)
{
  Controller c;
  ServoSender s("127.0.0.1", c.port(), opts(5, 4));
  ASSERT_TRUE(s.start(joint1(3.0)).ok());
  for (const double v : {10.0, 11.0, 12.0}) {ASSERT_TRUE(s.enqueue(joint1(v)));}  // already waiting
  ASSERT_TRUE(waitUntil([&] {return c.servoJCount() >= 7;}));
  s.stop();
  EXPECT_EQ(c.servoJFirstJoints(), (std::vector<double>{3, 3, 3, 3, 10, 11, 12}));
}

TEST(ServoSender, EveryCommandIsSentOnceInOrder)
{
  Controller c;
  ServoSender s("127.0.0.1", c.port(), opts(5));
  ASSERT_TRUE(s.start(joint1(0)).ok());
  std::vector<double> expected;
  for (int i = 1; i <= 50; ++i) {          // a burst far faster than the period
    ASSERT_TRUE(s.enqueue(joint1(i)));
    expected.push_back(i);
  }
  ASSERT_TRUE(waitUntil([&] {return c.servoJCount() >= 50;}));
  s.stop();
  EXPECT_EQ(c.servoJFirstJoints(), expected);   // none skipped, none repeated
  EXPECT_EQ(s.stats().overflows, 0u);
}

TEST(ServoSender, SlowCallsAreCaughtUpNotSkipped)
{
  Controller c;
  c.delayServoJ(20);                         // each call takes 4 periods
  ServoSender s("127.0.0.1", c.port(), opts(5));
  ASSERT_TRUE(s.start(joint1(0)).ok());
  std::vector<double> expected;
  for (int i = 1; i <= 15; ++i) {            // producer keeps its 5 ms rhythm regardless
    ASSERT_TRUE(s.enqueue(joint1(i)));
    expected.push_back(i);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(waitUntil([&] {return c.servoJCount() >= 15;}, 3000));
  s.stop();
  EXPECT_EQ(c.servoJFirstJoints(), expected);
  EXPECT_GE(s.stats().max_backlog, 2u);      // it did fall behind, and recovered the order
}

TEST(ServoSender, FullHostQueueRefusesAndCounts)
{
  Controller c;
  c.delayServoJ(2);                          // slower than the burst below, which takes microseconds
  ServoSender s("127.0.0.1", c.port(), opts(5));
  ASSERT_TRUE(s.start(joint1(0)).ok());
  int accepted = 0;
  std::vector<double> expected;
  for (int i = 1; i <= 120; ++i) {
    if (s.enqueue(joint1(i))) {++accepted; expected.push_back(i);}
  }
  EXPECT_GE(accepted, 60);
  EXPECT_LE(accepted, 68);                   // capacity 64 (+ the few the thread already took)
  EXPECT_EQ(s.stats().overflows, static_cast<uint64_t>(120 - accepted));
  ASSERT_TRUE(waitUntil([&] {return c.servoJCount() >= accepted;}, 5000));
  s.stop();
  EXPECT_EQ(c.servoJFirstJoints(), expected);  // what was accepted is delivered completely
}

TEST(ServoSender, HaltDiscardsWhatIsQueuedAndSendsNothingMore)
{
  Controller c;
  c.delayServoJ(50);                         // keeps the sender busy so commands wait on the host
  ServoSender s("127.0.0.1", c.port(), opts(5, 2));
  ASSERT_TRUE(s.start(joint1(0)).ok());
  for (int i = 1; i <= 10; ++i) {ASSERT_TRUE(s.enqueue(joint1(i)));}
  std::this_thread::sleep_for(20ms);         // the first prefill call is in flight
  s.halt();
  ASSERT_TRUE(waitUntil([&] {return s.stats().suppressed >= 10;}));
  std::this_thread::sleep_for(150ms);
  const int frozen = c.servoJCount();
  EXPECT_LE(frozen, 2);                      // at most the prefill calls that were already going
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(c.servoJCount(), frozen);        // nothing is sent while halted
  for (const double v : {18.0, 19.0, 20.0}) {s.enqueue(joint1(v));}
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(c.servoJCount(), frozen);        // nor anything queued later
  s.stop();
  for (const double x : c.servoJFirstJoints()) {EXPECT_EQ(x, 0);}  // only the prefill got out
}

TEST(ServoSender, ControllerRejectionIsCountedNotFatal)
{
  Controller c;
  c.setServoJAnswer(112);
  ServoSender s("127.0.0.1", c.port(), opts(5));
  ASSERT_TRUE(s.start(joint1(0)).ok());
  for (int i = 0; i < 5; ++i) {s.enqueue(joint1(i));}
  ASSERT_TRUE(waitUntil([&] {return s.stats().rejected >= 5;}));
  EXPECT_TRUE(s.running());                  // an error code is not a communication failure
  EXPECT_EQ(s.commError(), ApiError::None);
  EXPECT_EQ(s.stats().last_rejection, 112);
  EXPECT_EQ(s.stats().sent, 0u);
  s.stop();
}

TEST(ServoSender, ConnectionLossEndsTheThreadWithoutRetry)
{
  Controller c;
  c.closeAfterServoJ(3);
  ServoSender s("127.0.0.1", c.port(), opts(5));
  ASSERT_TRUE(s.start(joint1(0)).ok());
  for (int i = 0; i < 10; ++i) {s.enqueue(joint1(i));}
  ASSERT_TRUE(waitUntil([&] {return !s.running();}));
  EXPECT_NE(s.commError(), ApiError::None);
  EXPECT_EQ(c.servoJCount(), 3);             // the failed call is not repeated
  EXPECT_FALSE(s.enqueue(joint1(1)));            // the realtime side learns it from the return value
  EXPECT_EQ(s.stop().error, ApiError::NotConnected);
}

TEST(ServoSender, StartFailsWhenServoMoveStartIsRefused)
{
  Controller c;
  c.setStartAnswer(7);
  ServoSender s("127.0.0.1", c.port(), opts(5, 2));
  const auto st = s.start(joint1(0));
  EXPECT_EQ(st.error, ApiError::Controller);
  EXPECT_EQ(st.code, 7);
  EXPECT_FALSE(s.running());
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(c.servoJCount(), 0);             // not even the prefill
}

TEST(ServoSender, StartFailsWithoutController)
{
  uint16_t port;
  {
    Controller c;
    port = c.port();
  }
  ServoSender s("127.0.0.1", port, opts(5));
  EXPECT_FALSE(s.start(joint1(0)).ok());
  EXPECT_FALSE(s.running());
}

TEST(ServoSender, BadInputsAreRefused)
{
  Controller c;
  EXPECT_THROW(ServoSender("127.0.0.1", c.port(), opts(0)).start(joint1(0)), std::invalid_argument);
  EXPECT_THROW(ServoSender("127.0.0.1", c.port(), opts(1001)).start(joint1(0)), std::invalid_argument);
  EXPECT_THROW(ServoSender("127.0.0.1", c.port(), opts(5, 65)).start(joint1(0)), std::invalid_argument);
  EXPECT_THROW(ServoSender("127.0.0.1", c.port(), opts(5, -1)).start(joint1(0)), std::invalid_argument);
  ServoSender s("127.0.0.1", c.port(), opts(5));
  EXPECT_THROW(s.start({0, 0, std::nan(""), 0, 0, 0}), std::invalid_argument);

  EXPECT_FALSE(s.enqueue(joint1(1)));            // not running yet
  ASSERT_TRUE(s.start(joint1(4)).ok());
  EXPECT_FALSE(s.enqueue({std::nan(""), 0, 0, 0, 0, 0}));
  EXPECT_FALSE(s.enqueue({0, 0, 0, std::numeric_limits<double>::infinity(), 0, 0}));
  for (const double v : {5.0, 6.0, 7.0}) {ASSERT_TRUE(s.enqueue(joint1(v)));}
  ASSERT_TRUE(waitUntil([&] {return c.servoJCount() >= 3;}));
  s.stop();
  EXPECT_EQ(c.servoJFirstJoints(), (std::vector<double>{5, 6, 7}));  // the bad ones never went out
}

TEST(ServoSender, CallTimingIsMeasured)
{
  Controller c;
  c.delayServoJ(20);
  ServoSender s("127.0.0.1", c.port(), opts(5));
  ASSERT_TRUE(s.start(joint1(0)).ok());
  for (int i = 0; i < 4; ++i) {s.enqueue(joint1(i));}
  ASSERT_TRUE(waitUntil([&] {return s.stats().calls >= 4;}, 3000));
  s.stop();
  const auto st = s.stats();
  EXPECT_GE(st.mean_call_us, 18000.0);
  EXPECT_GE(st.max_call_us, 19000u);
}

// ---- drift compensation ----------------------------------------------------------------------

// The loop alone, on simulated time. Production settings: cmdT 8 ms, target depth 3, 0.1 rad/s.
namespace
{
constexpr double kCmdTs = 0.008;
constexpr int kTarget = 3;
constexpr double kBandwidth = 0.1;
constexpr double kSettleS = 0.3 / kBandwidth;  // the depth after a prefill is ignored this long

// Feeds `depth` every cmdT for `seconds`; returns the last eps.
double feed(DepthLoop & loop, int depth, double seconds)
{
  double eps = loop.eps();
  for (double t = 0; t < seconds; t += kCmdTs) {
    eps = loop.update(depth, kCmdTs);
  }
  return eps;
}
}  // namespace

TEST(DepthLoop, IgnoresTheDepthRightAfterAPrefill)
{
  DepthLoop loop(kCmdTs, kTarget, kBandwidth);
  EXPECT_EQ(feed(loop, 255, kSettleS - 2 * kCmdTs), 0.0);
}

TEST(DepthLoop, LeavesEpsAloneAtTheTargetDepth)
{
  DepthLoop loop(kCmdTs, kTarget, kBandwidth);
  EXPECT_EQ(feed(loop, kTarget, kSettleS + 60), 0.0);
}

TEST(DepthLoop, SendsSlowerWhenTheQueueIsDeepAndFasterWhenItIsShallow)
{
  DepthLoop deep(kCmdTs, kTarget, kBandwidth);
  EXPECT_GT(feed(deep, kTarget + 2, kSettleS + 1), 0.0);
  DepthLoop shallow(kCmdTs, kTarget, kBandwidth);
  EXPECT_LT(feed(shallow, kTarget - 2, kSettleS + 1), 0.0);
}

TEST(DepthLoop, EpsIsLimitedToOnePercent)
{
  DepthLoop loop(kCmdTs, kTarget, kBandwidth);
  EXPECT_EQ(feed(loop, 255, kSettleS + 60), DepthLoop::kMaxTrimPpm * 1e-6);
  DepthLoop empty(kCmdTs, kTarget, kBandwidth);
  EXPECT_EQ(feed(empty, 0, kSettleS + 600), -DepthLoop::kMaxTrimPpm * 1e-6);
}

TEST(DepthLoop, ARestartKeepsEps)
{
  DepthLoop loop(kCmdTs, kTarget, kBandwidth);
  const double eps = feed(loop, kTarget + 2, kSettleS + 5);
  ASSERT_GT(eps, 0.0);
  loop.restart();
  EXPECT_EQ(feed(loop, 0, kSettleS - 2 * kCmdTs), eps);
}

// The loop closed over a model of the firmware queue whose clock is `drift_ppm` slower than the
// host's (S17: about +1000 ppm on the real controller). Every send adds a segment; in the time to
// the next send, cmdT * (1 + eps), the controller plays cmdT * (1 + eps) / (cmdT * (1 + drift))
// segments. The queue starts at the target, as after a prefill. Drifts: the measured one and three
// times it, both signs. Limit: starting from eps = 0, a controller 4000 ppm or more faster than the
// host empties the queue before the loop has caught up (TODO.md).
class DriftCompensation : public ::testing::TestWithParam<double> {};

TEST_P(DriftCompensation, KeepsTheQueueAtTheTargetDepth)
{
  const double drift = GetParam() * 1e-6;
  DepthLoop loop(kCmdTs, kTarget, kBandwidth);
  double level = kTarget;
  int min_depth = kTarget;
  int min_settled = kTarget;
  int max_settled = kTarget;
  double eps = 0;
  for (double t = 0; t < 600; ) {
    const int depth = static_cast<int>(std::ceil(level));
    min_depth = std::min(min_depth, depth);
    if (t > 120) {  // a few time constants of the loop (1 / bandwidth = 10 s)
      min_settled = std::min(min_settled, depth);
      max_settled = std::max(max_settled, depth);
    }
    const double dt = kCmdTs * (1.0 + eps);
    eps = loop.update(depth, dt);
    level = std::max(0.0, level + 1.0 - dt / (kCmdTs * (1.0 + drift)));
    t += dt;
  }
  EXPECT_GE(min_depth, 1) << "the firmware queue ran empty";
  EXPECT_GE(min_settled, kTarget - 1);
  EXPECT_LE(max_settled, kTarget + 1);
  EXPECT_NEAR(eps, drift, 1e-4) << "eps follows the drift";
}

INSTANTIATE_TEST_SUITE_P(Drift, DriftCompensation, ::testing::Values(1000.0, -1000.0, 3000.0, -3000.0));

// A straight ramp cannot tell interpolation from extrapolation, a zigzag can: every sent point
// must lie between the commands it interpolates, so inside [0, 1].
TEST(ServoSender, TrimmedSendsStayOnThePolylineOfTheCommands)
{
  Controller c;
  ServoSenderOptions o = opts(2, 3);
  o.queue_depth = []() -> std::optional<int> {return 200;};   // far too deep: the trim goes to +1 %
  o.loop_bandwidth_rad_s = 4.0;                               // and gets there right after the settling
  ServoSender s("127.0.0.1", c.port(), o);
  ASSERT_TRUE(s.start(joint1(0)).ok());
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 500; ++i) {            // at 1 % the fraction wraps every 100 sends
    std::this_thread::sleep_until(t0 + 2ms * i);
    ASSERT_TRUE(s.enqueue(joint1(i % 2)));
  }
  EXPECT_DOUBLE_EQ(s.stats().trim_ppm, DepthLoop::kMaxTrimPpm);
  s.stop();
  for (const double v : c.servoJFirstJoints()) {
    EXPECT_GE(v, -1e-6);
    EXPECT_LE(v, 1.0 + 1e-6);
  }
  EXPECT_LE(s.stats().max_backlog, 5u);      // the sender keeps up with the producer in the long run
}

TEST(ServoSender, RealtimePriorityIsOptionalAndNeverFatal)
{
  Controller c;
  ServoSenderOptions o = opts(5);
  ServoSender plain("127.0.0.1", c.port(), o);
  ASSERT_TRUE(plain.start(joint1(0)).ok());
  EXPECT_FALSE(plain.realtimeGranted());
  plain.stop();

  o.rt_priority = 10;                        // granted or refused depending on the host: both work
  ServoSender rt("127.0.0.1", c.port(), o);
  ASSERT_TRUE(rt.start(joint1(0)).ok());
  for (int i = 0; i < 5; ++i) {ASSERT_TRUE(rt.enqueue(joint1(i)));}
  ASSERT_TRUE(waitUntil([&] {return rt.stats().sent >= 5;}));
  rt.stop();
  std::cout << "SCHED_FIFO granted: " << rt.realtimeGranted() << "\n";

  o.rt_priority = 100;
  EXPECT_THROW(ServoSender("127.0.0.1", c.port(), o).start(joint1(0)), std::invalid_argument);
}

TEST(ServoSender, WithoutDepthInformationTheTrimStaysZeroAndTheStreamIsUntouched)
{
  Controller c;
  ServoSenderOptions o = opts(2, 1);
  o.queue_depth = []() -> std::optional<int> {return std::nullopt;};
  ServoSender s("127.0.0.1", c.port(), o);
  ASSERT_TRUE(s.start(joint1(0)).ok());
  std::vector<double> expected{0};
  for (int i = 0; i < 30; ++i) {
    ASSERT_TRUE(s.enqueue(joint1(i)));
    expected.push_back(i);
  }
  ASSERT_TRUE(waitUntil([&] {return c.servoJCount() >= 31;}));
  s.stop();
  EXPECT_EQ(s.stats().trim_ppm, 0.0);
  EXPECT_EQ(c.servoJFirstJoints(), expected);
}

// Opt-in: FR_SIM_IP=192.168.58.2. Moves joint 1 of the simulated robot by a few degrees and back.
TEST(Simulator, ServoJMovesJointOneAndBack)
{
  const char * ip = std::getenv("FR_SIM_IP");
  if (ip == nullptr) {
    GTEST_SKIP() << "FR_SIM_IP not set";
  }
  StateReceiver receiver(ip);
  ASSERT_TRUE(receiver.start(kTargetVersion));
  ASSERT_TRUE(receiver.waitForFrame(2000));
  StateSnapshot snap;
  ASSERT_TRUE(receiver.latest(snap));
  ASSERT_EQ(snap.state.main_code, 0) << "controller reports a fault";
  ASSERT_EQ(snap.state.EmergencyStop, 0);
  ASSERT_EQ(snap.state.rbtEnableState, 1) << "robot is not enabled";

  JointArray home;
  for (int i = 0; i < 6; ++i) {home[i] = snap.state.jt_cur_pos[i];}
  const auto position = [&] {
      std::this_thread::sleep_for(150ms);   // the prefill delays the motion by prefill * cmdT
      receiver.latest(snap);
      return snap.state.jt_cur_pos[0];
    };

  ServoSender sender(ip);   // default prefill 3
  const auto started = sender.start(home);
  ASSERT_TRUE(started.ok()) << "ServoMoveStart: error " << static_cast<int>(started.error)
                            << " code " << started.code << " " << started.message;

  int max_queue = 0;
  const auto ramp = [&](double from, double to) {
      const int steps = static_cast<int>(std::abs(to - from) / 0.1);  // 0.1 deg per 8 ms
      auto next = std::chrono::steady_clock::now();
      for (int i = 1; i <= steps; ++i) {
        JointArray t = home;
        t[0] = from + (to - from) * i / steps;
        EXPECT_TRUE(sender.enqueue(t));
        next += 8ms;
        std::this_thread::sleep_until(next);
        receiver.latest(snap);
        max_queue = std::max(max_queue, snap.state.mc_queue_len);
      }
    };

  ramp(home[0], home[0] + 8.0);
  const double moved = position();
  ramp(home[0] + 8.0, home[0]);
  const double back = position();
  const auto ended = sender.stop();
  const auto stats = sender.stats();

  std::cout << "joint1 home=" << home[0] << " moved=" << moved << " back=" << back
            << "  sent=" << stats.sent << " rejected=" << stats.rejected
            << " max_backlog=" << stats.max_backlog << " max_mc_queue_len=" << max_queue
            << " mean_call_us=" << stats.mean_call_us << " ServoMoveEnd code=" << ended.code << "\n";
  EXPECT_EQ(sender.commError(), ApiError::None);
  EXPECT_EQ(stats.rejected, 0u);
  EXPECT_EQ(stats.overflows, 0u);
  EXPECT_NEAR(moved, home[0] + 8.0, 1.0);
  EXPECT_NEAR(back, home[0], 0.5);
  EXPECT_LE(max_queue, 10) << "firmware queue should stay near the prefill depth";
}
