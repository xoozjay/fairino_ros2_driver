#include "fr_client_library/frame_parser.hpp"

#include <algorithm>
#include <cstring>

namespace fr_client_library
{
namespace
{
// Larger than any known 20004 frame (max 1236); anything bigger is a garbage header.
constexpr std::size_t kMaxFrameSize = 4096;
constexpr std::size_t kHeaderSize = 5;

uint8_t u8(char c) {return static_cast<uint8_t>(c);}
}  // namespace

void FrameParser::feed(const char * data, std::size_t size, std::vector<RobotState384> & out)
{
  buf_.append(data, size);
  std::size_t pos = 0;

  for (;;) {
    // find the next 5A 5A
    std::size_t h = pos;
    while (h + 1 < buf_.size() && !(u8(buf_[h]) == 0x5A && u8(buf_[h + 1]) == 0x5A)) {
      ++h;
    }
    if (h + 1 >= buf_.size()) {
      // no header; keep a trailing 5A that may be the first half of one
      pos = (h < buf_.size() && u8(buf_[h]) == 0x5A) ? h : buf_.size();
      break;
    }
    pos = h;
    if (buf_.size() - pos < kHeaderSize) {
      break;
    }

    const std::size_t total = (static_cast<std::size_t>(u8(buf_[pos + 3])) |
      static_cast<std::size_t>(u8(buf_[pos + 4])) << 8) + kFrameOverhead;
    if (total > kMaxFrameSize) {
      ++stats_.bad_frames;
      ++pos;
      continue;
    }
    if (buf_.size() - pos < total) {
      break;  // wait for the rest
    }

    uint16_t sum = 0;
    for (std::size_t i = 0; i < total - 2; ++i) {
      sum = static_cast<uint16_t>(sum + u8(buf_[pos + i]));
    }
    const uint16_t wire_sum = static_cast<uint16_t>(
      u8(buf_[pos + total - 2]) | u8(buf_[pos + total - 1]) << 8);
    if (sum != wire_sum) {
      ++stats_.bad_frames;
      ++pos;  // resync on the next header
      continue;
    }

    if (total != expected_size_) {
      ++stats_.wrong_size_frames;
      stats_.last_wrong_size = total;
      pos += total;
      continue;
    }

    RobotState384 state{};
    std::memcpy(&state, buf_.data() + pos, std::min(total - 2, sizeof(RobotState384) - 2));
    state.check_sum = wire_sum;
    out.push_back(state);
    pos += total;
  }

  buf_.erase(0, pos);
}

}  // namespace fr_client_library
