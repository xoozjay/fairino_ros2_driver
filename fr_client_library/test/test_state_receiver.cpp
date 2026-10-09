#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "fr_client_library/state_receiver.hpp"
#include "test_support.hpp"

using namespace fr_client_library;
using namespace fr_test;
using namespace std::chrono_literals;

namespace
{
StateReceiverOptions fast()
{
  StateReceiverOptions o;
  o.connect_timeout_ms = 500;
  o.poll_ms = 10;
  return o;
}
}  // namespace

TEST(StateReceiver, ConnectFailureReported)
{
  uint16_t port;
  {
    FakeServer s([](int) {});
    port = s.port();
  }
  StateReceiver r("127.0.0.1", port, fast());
  EXPECT_FALSE(r.start(kTargetVersion));
  EXPECT_FALSE(r.running());
}

TEST(StateReceiver, ReceivesFramesSplitAcrossWrites)
{
  FakeServer s([](int fd) {
      const std::string data = makeFrame(960, 0xAB, 1) + makeFrame(960, 0xAB, 2);
      writeAll(fd, data.substr(0, 400));
      std::this_thread::sleep_for(20ms);
      writeAll(fd, data.substr(400));
      std::this_thread::sleep_for(300ms);  // keep the connection up
    });
  StateReceiver r("127.0.0.1", s.port(), fast());
  StateSnapshot snap;
  EXPECT_FALSE(r.latest(snap));  // nothing yet
  ASSERT_TRUE(r.start(kTargetVersion));
  ASSERT_TRUE(r.waitForFrame(1000));
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(r.latest(snap));
  EXPECT_EQ(snap.state.frame_cnt, 2);
  EXPECT_EQ(snap.frame_count, 2u);
  EXPECT_EQ(snap.state.frame_head, 0x5A5A);
  EXPECT_LT(std::chrono::steady_clock::now() - snap.received_at, 1s);
  r.stop();
  EXPECT_FALSE(r.running());
}

// The realtime loop (tryLatest) and other threads (latest) read at the same time as frames arrive.
TEST(StateReceiver, AnyThreadReadsTheNewestFrame)
{
  FakeServer s([](int fd) {
      for (int i = 0; i < 100; ++i) {
        const std::string f = makeFrame(960, 0xAB, static_cast<uint8_t>(i));
        if (::send(fd, f.data(), f.size(), MSG_NOSIGNAL) < 0) {return;}  // the test ended first
        std::this_thread::sleep_for(2ms);
      }
    });
  StateReceiver r("127.0.0.1", s.port(), fast());
  StateSnapshot none;
  EXPECT_FALSE(r.latest(none));
  EXPECT_FALSE(r.tryLatest(none));
  ASSERT_TRUE(r.start(kTargetVersion));
  ASSERT_TRUE(r.waitForFrame(1000));

  std::atomic<bool> stop{false};
  std::atomic<int> got{0};
  std::thread realtime([&] {
      StateSnapshot x;
      uint64_t seen = 0;
      while (!stop) {
        if (r.tryLatest(x)) {
          EXPECT_GE(x.frame_count, seen);
          EXPECT_EQ(x.state.frame_head, 0x5A5A);
          seen = x.frame_count;
          ++got;
        }
      }
    });
  uint64_t last = 0;
  for (int i = 0; i < 50; ++i) {
    StateSnapshot snap;
    ASSERT_TRUE(r.latest(snap));
    EXPECT_GE(snap.frame_count, last);   // never goes backwards
    EXPECT_EQ(snap.state.frame_head, 0x5A5A);
    last = snap.frame_count;
    std::this_thread::sleep_for(2ms);
  }
  stop = true;
  realtime.join();
  EXPECT_GT(last, 1u);
  EXPECT_GT(got.load(), 0);
}

TEST(StateReceiver, ConnectionLostEndsThread)
{
  FakeServer s([](int fd) {writeAll(fd, makeFrame(960));});  // then the server closes
  StateReceiver r("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(r.start(kTargetVersion));
  ASSERT_TRUE(r.waitForFrame(1000));
  for (int i = 0; i < 100 && r.running(); ++i) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(r.running());
  StateSnapshot snap;
  EXPECT_TRUE(r.latest(snap));  // the last good state stays readable; its age tells it is stale
}

TEST(StateReceiver, OtherVersionFramesAreCountedNotDelivered)
{
  FakeServer s([](int fd) {
      writeAll(fd, makeFrame(1184) + makeFrame(1184));
      std::this_thread::sleep_for(300ms);
    });
  StateReceiver r("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(r.start(kTargetVersion));
  EXPECT_FALSE(r.waitForFrame(200));
  const auto st = r.stats();
  EXPECT_EQ(st.wrong_size_frames, 2u);
  EXPECT_STREQ(describeFrameSize(st.last_wrong_size), "3.8.6");
}

TEST(StateReceiver, RestartKeepsCounterMonotonic)
{
  FakeServer s([](int fd) {
      writeAll(fd, makeFrame(960));
      std::this_thread::sleep_for(200ms);
    });
  StateReceiver r("127.0.0.1", s.port(), fast());
  StateSnapshot a, b;
  ASSERT_TRUE(r.start(kTargetVersion));
  ASSERT_TRUE(r.waitForFrame(1000));
  r.stop();
  ASSERT_TRUE(r.latest(a));
  ASSERT_TRUE(r.start(kTargetVersion));
  ASSERT_TRUE(r.waitForFrame(1000));
  std::this_thread::sleep_for(30ms);
  ASSERT_TRUE(r.latest(b));
  EXPECT_GT(b.frame_count, a.frame_count);
}

TEST(StateReceiver, ARestartForgetsTheFramesFromBefore)
{
  std::atomic<int> connections{0};
  FakeServer s([&](int fd) {
      if (connections++ == 0) {writeAll(fd, makeFrame(960));}
      std::this_thread::sleep_for(200ms);
    });
  StateReceiver r("127.0.0.1", s.port(), fast());
  StateSnapshot snap;
  ASSERT_TRUE(r.start(kTargetVersion));
  ASSERT_TRUE(r.waitForFrame(1000));
  ASSERT_TRUE(r.latest(snap));
  ASSERT_TRUE(r.start(kTargetVersion));      // the controller is back but silent
  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(r.latest(snap));
}

// Opt-in: FR_SIM_IP=192.168.58.2
TEST(Simulator, StateStream)
{
  const char * ip = std::getenv("FR_SIM_IP");
  if (ip == nullptr) {
    GTEST_SKIP() << "FR_SIM_IP not set";
  }
  StateReceiver r(ip);
  ASSERT_TRUE(r.start(kTargetVersion));
  ASSERT_TRUE(r.waitForFrame(2000));
  std::this_thread::sleep_for(100ms);
  StateSnapshot snap;
  ASSERT_TRUE(r.latest(snap));
  EXPECT_EQ(snap.state.frame_head, 0x5A5A);
  EXPECT_EQ(snap.state.data_len, 953);
  EXPECT_GT(snap.frame_count, 1u);
  EXPECT_EQ(r.stats().bad_frames, 0u);
  std::cout << "sim joints[deg]: " << snap.state.jt_cur_pos[0] << " " << snap.state.jt_cur_pos[1]
            << " " << snap.state.jt_cur_pos[2] << "  frames=" << snap.frame_count << "\n";
}

TEST(StateReceiver, VersionWithoutKnownFrameSizeIsRefused)
{
  StateReceiver r("127.0.0.1", 1, fast());
  EXPECT_THROW(r.start(SoftwareVersion{3, 9, 5}), std::invalid_argument);
}
