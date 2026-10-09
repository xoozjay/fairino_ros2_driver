// Software version of the controller, as reported by XML-RPC GetSoftwareVersion.
//
// The authority is the "webVersion" string of that call (simulator: "v3.8.3"). The vendor's
// "controllerVersion" ("V3.8.8-LA") is another numbering and is only shown, never compared.
// Everything version dependent -- the whitelist and the state frame size -- is keyed by this
// version, so nothing is guessed from frame sizes. The protocol parts we use (state frame,
// XML-RPC calls, 8080 instant commands) barely change between versions (docs/sdk_quirks.md), so
// versions outside the whitelist are likely to work too; they are simply not verified.
#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace fr_client_library
{
// Components after the third (hardware reports 3.8.4.2) are bug-fix builds of the same version:
// parsing ignores them, so 3.8.4.2 *is* 3.8.4. The raw text stays in RobotVersionInfo.
struct SoftwareVersion
{
  int major = 0;
  int minor = 0;
  int patch = 0;
};

constexpr bool operator==(const SoftwareVersion & a, const SoftwareVersion & b)
{
  return a.major == b.major && a.minor == b.minor && a.patch == b.patch;
}
constexpr bool operator<(const SoftwareVersion & a, const SoftwareVersion & b)
{
  if (a.major != b.major) {return a.major < b.major;}
  if (a.minor != b.minor) {return a.minor < b.minor;}
  return a.patch < b.patch;
}
constexpr bool operator<=(const SoftwareVersion & a, const SoftwareVersion & b) {return !(b < a);}

// "v3.8.4", "V3.8.4", "3.8.4", and builds of it such as "v3.8.4.2" (the extra ".N" parts are
// ignored); anything else is nullopt.
std::optional<SoftwareVersion> parseSoftwareVersion(const std::string & text);
std::string toString(const SoftwareVersion & v);  // "3.8.4"

// The version this driver targets (RobotState384 is its layout).
inline constexpr SoftwareVersion kTargetVersion{3, 8, 4};

// Versions the driver has been checked against: 3.8.4 (target hardware, reports 3.8.4.2) and
// 3.8.3 (the simulator; no 3.8.4 simulator exists). Add a version here only after running
// against it.
inline constexpr SoftwareVersion kVerifiedVersions[] = {{3, 8, 3}, {3, 8, 4}};

constexpr bool isVerifiedVersion(const SoftwareVersion & v)
{
  for (const auto & k : kVerifiedVersions) {
    if (k == v) {
      return true;
    }
  }
  return false;
}

// State frame (TCP 20004) size per software version, from SDK tag comparison
// (docs/sdk_quirks.md). Only versions in these ranges are known; 3.7.7 is not (no SDK tag),
// and 3.9.5+ moved the state stream to port 20005.
struct StateFrameProfile
{
  SoftwareVersion first;
  SoftwareVersion last;
  std::size_t frame_size;
  const char * versions;  // for messages
};

inline constexpr StateFrameProfile kStateFrameProfiles[] = {
  {{3, 7, 6}, {3, 7, 6}, 888, "3.7.6"},
  {{3, 7, 8}, {3, 8, 1}, 902, "3.7.8~3.8.1"},
  {{3, 8, 2}, {3, 8, 2}, 954, "3.8.2"},
  {{3, 8, 3}, {3, 8, 5}, 960, "3.8.3~3.8.5"},
  {{3, 8, 6}, {3, 8, 6}, 1184, "3.8.6"},
  {{3, 8, 7}, {3, 9, 4}, 1236, "3.8.7~3.9.4"},
};

// 0 if the version is not in any range.
constexpr std::size_t stateFrameSize(const SoftwareVersion & v)
{
  for (const auto & p : kStateFrameProfiles) {
    if (p.first <= v && v <= p.last) {
      return p.frame_size;
    }
  }
  return 0;
}

// Which versions send frames of this size ("3.8.6"), or nullptr. For diagnostics only.
constexpr const char * describeFrameSize(std::size_t size)
{
  for (const auto & p : kStateFrameProfiles) {
    if (p.frame_size == size) {
      return p.versions;
    }
  }
  return nullptr;
}

}  // namespace fr_client_library
