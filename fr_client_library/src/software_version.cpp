#include "fr_client_library/software_version.hpp"

#include <charconv>

namespace fr_client_library
{
std::optional<SoftwareVersion> parseSoftwareVersion(const std::string & text)
{
  const char * p = text.data();
  const char * const end = p + text.size();
  if (p != end && (*p == 'v' || *p == 'V')) {
    ++p;
  }
  int part[3];
  for (int i = 0; i < 3; ++i) {
    const auto r = std::from_chars(p, end, part[i]);
    if (r.ec != std::errc() || part[i] < 0) {
      return std::nullopt;
    }
    p = r.ptr;
    if (i < 2) {
      if (p == end || *p != '.') {
        return std::nullopt;
      }
      ++p;
    }
  }
  while (p != end && *p == '.') {  // build numbers after the third part ("3.8.4.2"): same version
    int build;
    const auto r = std::from_chars(p + 1, end, build);
    if (r.ec != std::errc() || build < 0) {
      return std::nullopt;
    }
    p = r.ptr;
  }
  if (p != end) {
    return std::nullopt;
  }
  return SoftwareVersion{part[0], part[1], part[2]};
}

std::string toString(const SoftwareVersion & v)
{
  return std::to_string(v.major) + "." + std::to_string(v.minor) + "." + std::to_string(v.patch);
}

}  // namespace fr_client_library
