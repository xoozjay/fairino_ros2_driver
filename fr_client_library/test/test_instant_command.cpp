#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <future>
#include <string>
#include <thread>

#include "fr_client_library/instant_command.hpp"
#include "test_support.hpp"

using namespace fr_client_library;
using namespace fr_test;
using namespace std::chrono_literals;

namespace
{
// Reads until `terminator` has been received (or error/timeout).
std::string readFrame(int fd, const std::string & terminator = "/b/f")
{
  std::string data;
  char c[128];
  while (data.size() < terminator.size() ||
    data.compare(data.size() - terminator.size(), terminator.size(), terminator) != 0)
  {
    const ssize_t n = ::recv(fd, c, sizeof(c), 0);
    if (n <= 0) {
      break;
    }
    data.append(c, static_cast<std::size_t>(n));
  }
  return data;
}

const std::string kStopReply = "/f/bIII44III102III1III1III/b/f";

InstantCommandChannel::Options fastOptions()
{
  InstantCommandChannel::Options o;
  o.connect_timeout_ms = 500;
  o.reply_timeout_ms = 300;
  return o;
}
}  // namespace

// ---- encoding: compared against the SDK 3.8.4 format strings (robot.cpp) ----

TEST(Encode, MatchesSdkFormatString)
{
  // StopMotion: "%d" is absent from the format string, the counter is literally 44.
  EXPECT_EQ(encodeInstantCommand(InstantCommand::Stop), "/f/bIII44III102III4IIISTOPIII/b/f");
}

// ---- reply parsing ----

TEST(ReplyParser, SingleReply)
{
  InstantReplyParser p;
  std::vector<InstantReply> out;
  p.feed(kStopReply.data(), kStopReply.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].cnt, 44);
  EXPECT_EQ(out[0].cmd_id, 102);
  EXPECT_EQ(out[0].content, "1");
}

TEST(ReplyParser, ByteByByte)
{
  InstantReplyParser p;
  std::vector<InstantReply> out;
  for (char c : kStopReply) {
    p.feed(&c, 1, out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].cmd_id, 102);
}

TEST(ReplyParser, TwoRepliesInOneChunkWithGarbage)
{
  const std::string second = "/f/bIII0III103III2IIIokIII/b/f";
  const std::string data = "xx/f/" + kStopReply + "junk" + second;
  InstantReplyParser p;
  std::vector<InstantReply> out;
  p.feed(data.data(), data.size(), out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].cmd_id, 102);
  EXPECT_EQ(out[1].cmd_id, 103);
  EXPECT_EQ(out[1].content, "ok");
}

TEST(ReplyParser, ResyncsAfterMalformedFrame)
{
  const std::string data = "/f/bIIIxIII" + kStopReply;
  InstantReplyParser p;
  std::vector<InstantReply> out;
  p.feed(data.data(), data.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].cmd_id, 102);
}

TEST(ReplyParser, ContentLengthMismatchIsRejected)
{
  const std::string bad = "/f/bIII1III102III3IIIabIII/b/f";  // len 3, content "ab"
  const std::string data = bad + kStopReply;
  InstantReplyParser p;
  std::vector<InstantReply> out;
  p.feed(data.data(), data.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].cnt, 44);
}

// ---- channel against a fake controller ----

TEST(Channel, NotConnected)
{
  InstantCommandChannel ch("127.0.0.1", 1);
  EXPECT_EQ(ch.send(InstantCommand::Stop).status, InstantStatus::NotConnected);
}

TEST(Channel, ConnectRefused)
{
  uint16_t port;
  {
    FakeServer s([](int) {});
    port = s.port();
  }
  InstantCommandChannel ch("127.0.0.1", port, fastOptions());
  EXPECT_FALSE(ch.connect());
  EXPECT_FALSE(ch.connected());
}

TEST(Channel, StopGetsReply)
{
  std::promise<std::string> got;
  FakeServer s([&](int fd) {
      got.set_value(readFrame(fd));
      writeAll(fd, kStopReply);
    });
  InstantCommandChannel ch("127.0.0.1", s.port(), fastOptions());
  ASSERT_TRUE(ch.connect());
  const auto r = ch.send(InstantCommand::Stop);
  EXPECT_EQ(r.status, InstantStatus::Ok);
  EXPECT_EQ(r.content, "1");
  EXPECT_EQ(got.get_future().get(), "/f/bIII44III102III4IIISTOPIII/b/f");
  EXPECT_TRUE(ch.connected());
}

TEST(Channel, ReplySplitAndForeignReplySkipped)
{
  FakeServer s([&](int fd) {
      readFrame(fd);
      writeAll(fd, "/f/bIII0III103III1III1III/b/f");  // reply to another command
      writeAll(fd, "/f/bIII44III10");
      std::this_thread::sleep_for(50ms);
      writeAll(fd, "2III1III1III/b/f");
    });
  InstantCommandChannel ch("127.0.0.1", s.port(), fastOptions());
  ASSERT_TRUE(ch.connect());
  EXPECT_EQ(ch.send(InstantCommand::Stop).status, InstantStatus::Ok);
}

TEST(Channel, NoReplyTimesOutAndDisconnects)
{
  std::promise<bool> peer_saw_close;
  FakeServer s([&](int fd) {
      readFrame(fd);
      char c;
      peer_saw_close.set_value(::recv(fd, &c, 1, 0) == 0);  // EOF once the client gives up
    });
  InstantCommandChannel ch("127.0.0.1", s.port(), fastOptions());
  ASSERT_TRUE(ch.connect());
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(ch.send(InstantCommand::Stop).status, InstantStatus::Timeout);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1500ms);
  EXPECT_FALSE(ch.connected());
  EXPECT_TRUE(peer_saw_close.get_future().get());
  EXPECT_EQ(ch.send(InstantCommand::Stop).status, InstantStatus::NotConnected);
}

TEST(Channel, PeerClosedIsIoError)
{
  FakeServer s([&](int fd) {readFrame(fd);});
  InstantCommandChannel ch("127.0.0.1", s.port(), fastOptions());
  ASSERT_TRUE(ch.connect());
  EXPECT_EQ(ch.send(InstantCommand::Stop).status, InstantStatus::IoError);
  EXPECT_FALSE(ch.connected());
}

// Opt-in: FR_SIM_IP=192.168.58.2 sends STOP to the simulator (no motion command).
TEST(Simulator, StopGetsReply)
{
  const char * ip = std::getenv("FR_SIM_IP");
  if (ip == nullptr) {
    GTEST_SKIP() << "FR_SIM_IP not set";
  }
  InstantCommandChannel ch(ip);
  ASSERT_TRUE(ch.connect());
  const auto r = ch.send(InstantCommand::Stop);
  EXPECT_EQ(r.status, InstantStatus::Ok);
  std::cout << "STOP reply content: '" << r.content << "'\n";
}
