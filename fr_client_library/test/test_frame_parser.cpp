#include <gtest/gtest.h>

#include <cstring>
#include <random>
#include <string>

#include "fr_client_library/frame_parser.hpp"
#include "test_support.hpp"

using namespace fr_client_library;
using namespace fr_test;

namespace
{
std::vector<RobotState384> feedAll(FrameParser & p, const std::string & s)
{
  std::vector<RobotState384> out;
  p.feed(s.data(), s.size(), out);
  return out;
}
}  // namespace

TEST(FrameParser, DefaultAcceptsTargetVersionFrame)
{
  FrameParser p;
  std::string f = makeFrame(960, 0x00, 7);
  const int32_t code = 0x01020304;
  std::memcpy(&f[offsetof(RobotState384, main_code)], &code, sizeof(code));
  // re-sign after editing
  uint16_t sum = 0;
  for (std::size_t i = 0; i < 958; ++i) {sum = static_cast<uint16_t>(sum + static_cast<uint8_t>(f[i]));}
  std::memcpy(&f[958], &sum, 2);

  const auto out = feedAll(p, f);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].frame_head, 0x5A5A);
  EXPECT_EQ(out[0].frame_cnt, 7);
  EXPECT_EQ(out[0].data_len, 953);
  EXPECT_EQ(out[0].main_code, code);
  EXPECT_EQ(out[0].check_sum, sum);
}

TEST(FrameParser, ByteByByte)
{
  FrameParser p;
  const std::string f = makeFrame(960);
  std::vector<RobotState384> out;
  for (char c : f) {p.feed(&c, 1, out);}
  EXPECT_EQ(out.size(), 1u);
}

TEST(FrameParser, TwoFramesGarbageAndStrayHeaderByte)
{
  FrameParser p;
  const std::string data = std::string("\x01\x02\x5A", 3) + makeFrame(960, 0xAB, 1) +
    std::string("junk\x5A", 5) + makeFrame(960, 0xAB, 2);
  const auto out = feedAll(p, data);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].frame_cnt, 1);
  EXPECT_EQ(out[1].frame_cnt, 2);
}

TEST(FrameParser, RandomChunking)
{
  std::string stream;
  for (int i = 0; i < 6; ++i) {stream += makeFrame(960, 0xAB, static_cast<uint8_t>(i));}
  std::mt19937 rng(42);
  for (int round = 0; round < 20; ++round) {
    FrameParser p;
    std::vector<RobotState384> out;
    for (std::size_t pos = 0; pos < stream.size();) {
      const std::size_t n = std::min<std::size_t>(1 + rng() % 700, stream.size() - pos);
      p.feed(stream.data() + pos, n, out);
      pos += n;
    }
    ASSERT_EQ(out.size(), 6u);
    EXPECT_EQ(out[5].frame_cnt, 5);
  }
}

TEST(FrameParser, BadChecksumRejectedAndResynced)
{
  FrameParser p;
  std::string bad = makeFrame(960, 0xAB, 1);
  bad[100] ^= 0x01;
  const auto out = feedAll(p, bad + makeFrame(960, 0xAB, 2));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].frame_cnt, 2);
  EXPECT_GE(p.stats().bad_frames, 1u);
}

TEST(FrameParser, ImplausibleLengthDoesNotStall)
{
  FrameParser p;
  const std::string data = std::string("\x5A\x5A\x01\xFF\xFF", 5) + makeFrame(960, 0xAB, 3);
  const auto out = feedAll(p, data);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].frame_cnt, 3);
}

TEST(FrameParser, OtherVersionFrameIsRejectedAndIdentified)
{
  FrameParser p;  // expects the target version
  const auto out = feedAll(p, makeFrame(1184) + makeFrame(960, 0xAB, 9));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].frame_cnt, 9);
  EXPECT_EQ(p.stats().wrong_size_frames, 1u);
  EXPECT_STREQ(describeFrameSize(p.stats().last_wrong_size), "3.8.6");
}

TEST(FrameParser, NewerVersionKeepsKnownPrefix)
{
  FrameParser p(SoftwareVersion{3, 8, 6});
  const auto out = feedAll(p, makeFrame(1184, 0xAB, 4));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].data_len, 1177);  // real frame size stays visible
  EXPECT_EQ(static_cast<uint32_t>(out[0].main_code), 0xABABABABu);
  EXPECT_EQ(static_cast<uint32_t>(out[0].rbtEnableState), 0xABABABABu);
  EXPECT_EQ(feedAll(p, makeFrame(960)).size(), 0u);  // and 3.8.4 frames are now the foreign ones
}

TEST(FrameParser, OlderVersionZeroFillsMissingTail)
{
  FrameParser p(SoftwareVersion{3, 8, 2});  // 954 bytes: payload ends before wideVoltageCtrlBoxFanCurrent
  const auto out = feedAll(p, makeFrame(954, 0xAB, 5));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].data_len, 947);
  EXPECT_EQ(static_cast<uint32_t>(out[0].rbtEnableState), 0xABABABABu);
  EXPECT_EQ(out[0].wideVoltageCtrlBoxFanCurrent, 0);
}

TEST(FrameParser, ThreeEightThreeUsesTheSameFrameAsThreeEightFour)
{
  FrameParser p(SoftwareVersion{3, 8, 3});  // the simulator
  EXPECT_EQ(feedAll(p, makeFrame(960, 0xAB, 3)).size(), 1u);
}

TEST(FrameParser, VersionWithoutKnownFrameSizeIsRefused)
{
  EXPECT_THROW(FrameParser{SoftwareVersion({3, 7, 7})}, std::invalid_argument);
  EXPECT_THROW(FrameParser{SoftwareVersion({3, 9, 5})}, std::invalid_argument);
}
