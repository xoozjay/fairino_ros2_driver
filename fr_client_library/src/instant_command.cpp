#include "fr_client_library/instant_command.hpp"

#include <chrono>

namespace fr_client_library
{
namespace
{
using Clock = std::chrono::steady_clock;

constexpr char kSep[] = "III";
constexpr char kHead[] = "/f/b";
constexpr char kTail[] = "/b/f";
constexpr std::size_t kMaxContent = 4096;

enum class Parse { Ok, Incomplete, Bad };

// Does `lit` appear at buf[pos]? Incomplete if buf ends while still matching.
Parse matchLiteral(const std::string & buf, std::size_t pos, const std::string & lit)
{
  for (std::size_t i = 0; i < lit.size(); ++i) {
    if (pos + i >= buf.size()) {
      return Parse::Incomplete;
    }
    if (buf[pos + i] != lit[i]) {
      return Parse::Bad;
    }
  }
  return Parse::Ok;
}

// Reads decimal digits at buf[pos] followed by "III".
Parse readNumber(const std::string & buf, std::size_t & pos, int & value)
{
  std::size_t i = pos;
  long v = 0;
  while (i < buf.size() && buf[i] >= '0' && buf[i] <= '9') {
    v = v * 10 + (buf[i] - '0');
    if (++i - pos > 9) {
      return Parse::Bad;
    }
  }
  if (i == pos) {
    return i >= buf.size() ? Parse::Incomplete : Parse::Bad;
  }
  const Parse sep = matchLiteral(buf, i, kSep);
  if (sep != Parse::Ok) {
    return sep;
  }
  value = static_cast<int>(v);
  pos = i + 3;
  return Parse::Ok;
}

// Parses one reply starting at buf[0] == "/f/b"; on Ok, `consumed` is its length.
Parse parseReply(const std::string & buf, InstantReply & reply, std::size_t & consumed)
{
  std::size_t pos = 4;
  int len = 0;
  Parse r = matchLiteral(buf, pos, kSep);
  if (r != Parse::Ok) {
    return r;
  }
  pos += 3;
  if ((r = readNumber(buf, pos, reply.cnt)) != Parse::Ok) {return r;}
  if ((r = readNumber(buf, pos, reply.cmd_id)) != Parse::Ok) {return r;}
  if ((r = readNumber(buf, pos, len)) != Parse::Ok) {return r;}
  if (static_cast<std::size_t>(len) > kMaxContent) {
    return Parse::Bad;
  }
  const std::size_t content_end = pos + static_cast<std::size_t>(len);
  if (buf.size() < content_end) {
    return Parse::Incomplete;
  }
  reply.content = buf.substr(pos, static_cast<std::size_t>(len));
  r = matchLiteral(buf, content_end, std::string(kSep) + kTail);
  if (r != Parse::Ok) {
    return r;
  }
  consumed = content_end + 3 + 4;
  return Parse::Ok;
}
}  // namespace

std::string encodeInstantCommand(InstantCommand cmd)
{
  // The SDK writes STOP's counter as the literal 44, not a running count.
  const std::string cnt = "44";
  const std::string content = "STOP";
  return std::string(kHead) + kSep + cnt + kSep + std::to_string(static_cast<int>(cmd)) + kSep +
         std::to_string(content.size()) + kSep + content + kSep + kTail;
}

void InstantReplyParser::feed(const char * data, std::size_t size, std::vector<InstantReply> & out)
{
  buf_.append(data, size);
  for (;;) {
    const std::size_t head = buf_.find(kHead);
    if (head == std::string::npos) {
      // keep a possible partial head at the end
      buf_.erase(0, buf_.size() > 3 ? buf_.size() - 3 : 0);
      return;
    }
    buf_.erase(0, head);
    InstantReply reply;
    std::size_t consumed = 0;
    switch (parseReply(buf_, reply, consumed)) {
      case Parse::Incomplete:
        return;
      case Parse::Bad:
        buf_.erase(0, 1);  // resync on the next head
        break;
      case Parse::Ok:
        out.push_back(std::move(reply));
        buf_.erase(0, consumed);
        break;
    }
  }
}

InstantCommandChannel::InstantCommandChannel(std::string ip, uint16_t port, Options options)
: ip_(std::move(ip)), port_(port), options_(options)
{
}

bool InstantCommandChannel::connect()
{
  parser_ = InstantReplyParser{};
  return sock_.connect(ip_, port_, options_.connect_timeout_ms);
}

InstantResult InstantCommandChannel::fail(InstantStatus status)
{
  sock_.close();
  return InstantResult{status, {}};
}

InstantResult InstantCommandChannel::send(InstantCommand cmd)
{
  if (!sock_.isOpen()) {
    return InstantResult{InstantStatus::NotConnected, {}};
  }
  const auto deadline = Clock::now() + std::chrono::milliseconds(options_.reply_timeout_ms);
  const auto remaining_ms = [&]() {
      const auto left =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
      return left > 0 ? static_cast<int>(left) : 0;
    };

  char buf[512];
  std::vector<InstantReply> replies;

  // Discard anything left over so it cannot be mistaken for our reply.
  parser_ = InstantReplyParser{};
  for (ssize_t n; (n = sock_.recvSome(buf, sizeof(buf), 0)) != 0;) {
    if (n < 0) {
      return fail(InstantStatus::IoError);
    }
  }

  const std::string frame = encodeInstantCommand(cmd);
  if (!sock_.sendAll(frame.data(), frame.size(), remaining_ms())) {
    return fail(InstantStatus::IoError);
  }

  for (;;) {
    const int left = remaining_ms();
    if (left == 0) {
      return fail(InstantStatus::Timeout);
    }
    const ssize_t n = sock_.recvSome(buf, sizeof(buf), left);
    if (n < 0) {
      return fail(InstantStatus::IoError);
    }
    replies.clear();
    parser_.feed(buf, static_cast<std::size_t>(n), replies);
    for (auto & reply : replies) {
      if (reply.cmd_id == static_cast<int>(cmd)) {
        return InstantResult{InstantStatus::Ok, std::move(reply.content)};
      }
    }
  }
}

}  // namespace fr_client_library
