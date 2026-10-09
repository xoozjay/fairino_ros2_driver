#include <gtest/gtest.h>

#include "fr_client_library/software_version.hpp"

using namespace fr_client_library;

TEST(SoftwareVersion, ParsesWhatTheControllerReports)
{
  EXPECT_EQ(*parseSoftwareVersion("v3.8.3"), (SoftwareVersion{3, 8, 3}));  // simulator's webVersion
  EXPECT_EQ(*parseSoftwareVersion("V3.8.4"), kTargetVersion);
  EXPECT_EQ(*parseSoftwareVersion("3.9.4"), (SoftwareVersion{3, 9, 4}));
  EXPECT_EQ(*parseSoftwareVersion("v10.0.12"), (SoftwareVersion{10, 0, 12}));
  EXPECT_EQ(toString(kTargetVersion), "3.8.4");
}

// The 3.8.4 hardware reports 3.8.4.2; builds after the third number are the same version.
TEST(SoftwareVersion, BuildNumbersAreTheSameVersion)
{
  for (const char * text : {"v3.8.4.2", "3.8.4.0", "V3.8.4.17", "3.8.4.2.7"}) {
    const auto v = parseSoftwareVersion(text);
    ASSERT_TRUE(v.has_value()) << text;
    EXPECT_EQ(*v, kTargetVersion) << text;
    EXPECT_TRUE(isVerifiedVersion(*v)) << text;
    EXPECT_EQ(stateFrameSize(*v), 960u) << text;
  }
  EXPECT_EQ(toString(*parseSoftwareVersion("v3.8.4.2")), "3.8.4");
  // a build number never turns another version into a known one
  EXPECT_FALSE(isVerifiedVersion(*parseSoftwareVersion("3.8.5.2")));
  EXPECT_EQ(stateFrameSize(*parseSoftwareVersion("3.7.7.1")), 0u);
}

TEST(SoftwareVersion, RejectsAnythingElse)
{
  // "V3.8.8-LA" is the vendor's controllerVersion: another numbering, must not parse as 3.8.8
  for (const char * bad : {"", "v", "V3.8.8-LA", "3.8", "3.8.4.", "3.8.4.2.", "3.8.4.-1", "3.8.4.2-rc", "v3..4", "3.8.x", " 3.8.4",
      "3.8.4 ", "vv3.8.4", "-3.8.4", "FR5-V1-002(V6.0)"})
  {
    EXPECT_FALSE(parseSoftwareVersion(bad).has_value()) << bad;
  }
}

TEST(SoftwareVersion, Ordering)
{
  EXPECT_TRUE((SoftwareVersion{3, 8, 3}) < (SoftwareVersion{3, 8, 4}));
  EXPECT_TRUE((SoftwareVersion{3, 8, 9}) < (SoftwareVersion{3, 9, 0}));
  EXPECT_TRUE((SoftwareVersion{2, 99, 99}) < (SoftwareVersion{3, 0, 0}));
  EXPECT_FALSE((SoftwareVersion{3, 8, 4}) < (SoftwareVersion{3, 8, 4}));
}

TEST(Whitelist, OnlyVerifiedVersions)
{
  static_assert(isVerifiedVersion({3, 8, 4}));
  static_assert(isVerifiedVersion({3, 8, 3}));  // the simulator
  static_assert(!isVerifiedVersion({3, 8, 5}));
  static_assert(!isVerifiedVersion({3, 7, 8}));
  static_assert(!isVerifiedVersion({}));
  SUCCEED();
}

TEST(FrameSize, MappedFromSoftwareVersion)
{
  EXPECT_EQ(stateFrameSize({3, 8, 4}), 960u);
  EXPECT_EQ(stateFrameSize({3, 8, 3}), 960u);  // same layout, both verified
  EXPECT_EQ(stateFrameSize({3, 8, 5}), 960u);
  EXPECT_EQ(stateFrameSize({3, 7, 6}), 888u);  // lower bound of the supported range
  EXPECT_EQ(stateFrameSize({3, 7, 8}), 902u);
  EXPECT_EQ(stateFrameSize({3, 8, 1}), 902u);
  EXPECT_EQ(stateFrameSize({3, 8, 2}), 954u);
  EXPECT_EQ(stateFrameSize({3, 8, 6}), 1184u);
  EXPECT_EQ(stateFrameSize({3, 9, 4}), 1236u);
}

TEST(FrameSize, UnknownVersionsHaveNoSize)
{
  EXPECT_EQ(stateFrameSize({3, 7, 7}), 0u);  // no SDK tag, not guessed
  EXPECT_EQ(stateFrameSize({3, 7, 5}), 0u);  // below the supported minimum
  EXPECT_EQ(stateFrameSize({3, 9, 5}), 0u);  // state stream moved to port 20005
  EXPECT_EQ(stateFrameSize({4, 0, 0}), 0u);
}

TEST(FrameSize, ProfilesAreOrderedAndDoNotOverlap)
{
  for (std::size_t i = 0; i < std::size(kStateFrameProfiles); ++i) {
    const auto & p = kStateFrameProfiles[i];
    EXPECT_TRUE(p.first <= p.last) << p.versions;
    EXPECT_EQ(toString(p.first), std::string(p.versions).substr(0, toString(p.first).size()));
    if (i > 0) {
      EXPECT_TRUE(kStateFrameProfiles[i - 1].last < p.first) << p.versions;
    }
  }
}

TEST(FrameSize, ReverseLookupForDiagnostics)
{
  EXPECT_STREQ(describeFrameSize(960), "3.8.3~3.8.5");
  EXPECT_STREQ(describeFrameSize(1184), "3.8.6");
  EXPECT_EQ(describeFrameSize(1), nullptr);
}
