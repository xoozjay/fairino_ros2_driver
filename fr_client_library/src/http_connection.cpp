#include "fr_client_library/http_connection.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>

namespace fr_client_library
{
namespace
{
using Clock = std::chrono::steady_clock;

// Reconnect when the connection has been idle for this much less than the server's timeout.
constexpr int kIdleMarginMs = 2000;
constexpr std::size_t kMaxHeaderSize = 8192;
constexpr std::size_t kMaxBodySize = 1 << 20;

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t");
  const auto e = s.find_last_not_of(" \t");
  return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

struct Header
{
  int status = 0;
  long content_length = -1;
  bool close = false;
  int keepalive_timeout_s = 0;
};

// `head` is everything before the blank line.
bool parseHeader(const std::string & head, Header & h)
{
  const auto eol = head.find("\r\n");
  const std::string status_line = head.substr(0, eol);
  if (status_line.compare(0, 5, "HTTP/") != 0) {
    return false;
  }
  const auto sp = status_line.find(' ');
  if (sp == std::string::npos ||
    std::from_chars(status_line.data() + sp + 1, status_line.data() + status_line.size(),
    h.status).ec != std::errc())
  {
    return false;
  }
  std::size_t pos = eol;
  while (pos != std::string::npos && pos < head.size()) {
    const std::size_t start = pos + 2;
    const std::size_t end = head.find("\r\n", start);
    const std::string line = head.substr(start, end == std::string::npos ? end : end - start);
    pos = end;
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string name = lower(trim(line.substr(0, colon)));
    const std::string value = lower(trim(line.substr(colon + 1)));
    if (name == "content-length") {
      long n = -1;
      if (std::from_chars(value.data(), value.data() + value.size(), n).ec != std::errc() ||
        n < 0)
      {
        return false;
      }
      h.content_length = n;
    } else if (name == "connection") {
      h.close = value.find("close") != std::string::npos;
    } else if (name == "keep-alive") {
      const auto t = value.find("timeout=");
      if (t != std::string::npos) {
        std::from_chars(value.data() + t + 8, value.data() + value.size(), h.keepalive_timeout_s);
      }
    }
  }
  return true;
}
}  // namespace

HttpConnection::HttpConnection(std::string ip, uint16_t port, HttpOptions options)
: ip_(std::move(ip)), port_(port), options_(options)
{
}

bool HttpConnection::connect()
{
  rotate_ = false;
  keepalive_timeout_ms_ = 0;
  if (!sock_.connect(ip_, port_, options_.connect_timeout_ms)) {
    return false;
  }
  last_activity_ = Clock::now();
  return true;
}

void HttpConnection::close()
{
  sock_.close();
  rotate_ = false;
}

HttpResult HttpConnection::fail(HttpStatus status)
{
  close();
  return HttpResult{status, {}};
}

bool HttpConnection::idleTooLong() const
{
  if (keepalive_timeout_ms_ <= 0) {
    return false;
  }
  const auto idle = std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now() - last_activity_).count();
  return idle >= std::max(0, keepalive_timeout_ms_ - kIdleMarginMs);
}

HttpResult HttpConnection::post(const std::string & body)
{
  if (sock_.isOpen() && idleTooLong()) {
    sock_.close();
    rotate_ = true;
  }
  if (!sock_.isOpen()) {
    if (!rotate_) {
      return HttpResult{HttpStatus::NotConnected, {}};
    }
    if (!connect()) {
      return fail(HttpStatus::IoError);
    }
  }

  const auto deadline = Clock::now() + std::chrono::milliseconds(options_.request_timeout_ms);
  const auto remaining_ms = [&]() {
      const auto left =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
      return left > 0 ? static_cast<int>(left) : 0;
    };

  // Same request head as the SDK's XmlRpc++ 0.7.
  const std::string request = "POST /RPC2 HTTP/1.1\r\nUser-Agent: XMLRPC++ 0.7\r\nHost: " + ip_ +
    ":" + std::to_string(port_) + "\r\nContent-Type: text/xml\r\nContent-length: " +
    std::to_string(body.size()) + "\r\n\r\n" + body;
  if (!sock_.sendAll(request.data(), request.size(), remaining_ms())) {
    return fail(HttpStatus::IoError);
  }

  std::string in;
  Header header;
  std::size_t body_start = 0;
  bool have_header = false;
  char buf[2048];
  for (;;) {
    if (!have_header) {
      const auto end = in.find("\r\n\r\n");
      if (end != std::string::npos) {
        if (!parseHeader(in.substr(0, end), header) || header.status != 200 ||
          header.content_length < 0 || static_cast<std::size_t>(header.content_length) > kMaxBodySize)
        {
          return fail(HttpStatus::ProtocolError);
        }
        have_header = true;
        body_start = end + 4;
      } else if (in.size() > kMaxHeaderSize) {
        return fail(HttpStatus::ProtocolError);
      }
    }
    if (have_header && in.size() - body_start >= static_cast<std::size_t>(header.content_length)) {
      break;
    }
    const int left = remaining_ms();
    if (left == 0) {
      return fail(HttpStatus::Timeout);
    }
    const ssize_t n = sock_.recvSome(buf, sizeof(buf), left);
    if (n < 0) {
      return fail(HttpStatus::IoError);
    }
    in.append(buf, static_cast<std::size_t>(n));
  }

  HttpResult result{HttpStatus::Ok, in.substr(body_start, static_cast<std::size_t>(header.content_length))};
  if (header.keepalive_timeout_s > 0) {
    keepalive_timeout_ms_ = header.keepalive_timeout_s * 1000;
  }
  if (header.close) {
    sock_.close();
    rotate_ = true;
  } else {
    last_activity_ = Clock::now();
  }
  return result;
}

}  // namespace fr_client_library
