// Reassembles state frames (TCP 20004) from a byte stream.
//
// Frame: 5A 5A | frame_cnt(1) | data_len(2, LE) | ... | checksum(2, LE); size = data_len + 7;
// checksum = u16 sum of all preceding bytes. The parser is created for a software version
// (from GetSoftwareVersion) and accepts exactly that version's frame size, see
// software_version.hpp. Default is the target 3.8.4.
#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "fr_client_library/robot_state_384.hpp"
#include "fr_client_library/software_version.hpp"

namespace fr_client_library
{
static_assert(stateFrameSize(kTargetVersion) == kRobotState384Size, "target = RobotState384");

class FrameParser
{
public:
  struct Stats
  {
    uint64_t bad_frames = 0;         // garbage header or checksum mismatch
    uint64_t wrong_size_frames = 0;  // valid frames of a size other than the expected one
    std::size_t last_wrong_size = 0;  // look it up with describeFrameSize()
  };

  // Throws std::invalid_argument for a version without a known state frame size.
  explicit FrameParser(const SoftwareVersion & version = kTargetVersion)
  : expected_size_(stateFrameSize(version))
  {
    if (expected_size_ == 0) {
      throw std::invalid_argument("no state frame size known for software version " +
        toString(version));
    }
  }

  // Appends every complete, valid frame found to `out`. Fields the version does not have are
  // zero (state.data_len tells the real size); extra trailing fields of newer versions are
  // dropped. State is never trusted blindly: all-zero is not a valid state (docs R3).
  void feed(const char * data, std::size_t size, std::vector<RobotState384> & out);

  const Stats & stats() const { return stats_; }

private:
  std::size_t expected_size_;
  std::string buf_;
  Stats stats_;
};

}  // namespace fr_client_library
