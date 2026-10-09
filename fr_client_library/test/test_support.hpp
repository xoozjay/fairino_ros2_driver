// Shared helpers for the tests: a loopback fake controller and frame builders.
#pragma once

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "fr_client_library/robot_state_384.hpp"

namespace fr_test
{
// Listens on 127.0.0.1 (ephemeral port) and runs `handler(fd)` for each accepted connection,
// one after another. The destructor stops it; destroy clients first.
class FakeServer
{
public:
  explicit FakeServer(std::function<void(int)> handler)
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    EXPECT_EQ(::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
    socklen_t len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    EXPECT_EQ(::listen(listen_fd_, 4), 0);
    thread_ = std::thread([this, handler] {
        for (;;) {
          const int fd = ::accept(listen_fd_, nullptr, nullptr);
          if (fd < 0) {
            return;
          }
          timeval tv{2, 0};  // never hang a test
          ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
          handler(fd);
          ::close(fd);
        }
      });
  }
  ~FakeServer()
  {
    ::shutdown(listen_fd_, SHUT_RDWR);
    thread_.join();
    ::close(listen_fd_);
  }
  uint16_t port() const {return port_;}

private:
  int listen_fd_;
  uint16_t port_;
  std::thread thread_;
};

inline void writeAll(int fd, const std::string & s)
{
  EXPECT_EQ(::send(fd, s.data(), s.size(), MSG_NOSIGNAL), static_cast<ssize_t>(s.size()));
}

// Reads one HTTP request; returns its body ("" on error/EOF). `head` gets the request head.
inline std::string readHttpRequest(int fd, std::string * head = nullptr)
{
  std::string in;
  char c[512];
  std::size_t end = std::string::npos;
  while ((end = in.find("\r\n\r\n")) == std::string::npos) {
    const ssize_t n = ::recv(fd, c, sizeof(c), 0);
    if (n <= 0) {
      return {};
    }
    in.append(c, static_cast<std::size_t>(n));
  }
  const std::string h = in.substr(0, end);
  if (head) {
    *head = h;
  }
  const auto cl = h.find("Content-length: ");
  const std::size_t len = cl == std::string::npos ? 0 : std::stoul(h.substr(cl + 16));
  while (in.size() < end + 4 + len) {
    const ssize_t n = ::recv(fd, c, sizeof(c), 0);
    if (n <= 0) {
      return {};
    }
    in.append(c, static_cast<std::size_t>(n));
  }
  return in.substr(end + 4, len);
}

// Response in the shape the controller's server (Abyss) sends.
inline std::string httpResponse(
  const std::string & body, const std::string & connection = "Keep-Alive",
  const std::string & keep_alive = "timeout=15, max=30")
{
  std::string r = "HTTP/1.1 200 OK\r\nContent-type: text/xml; charset=utf-8\r\nContent-length: " +
    std::to_string(body.size()) + "\r\nConnection: " + connection + "\r\n";
  if (!keep_alive.empty()) {
    r += "Keep-Alive: " + keep_alive + "\r\n";
  }
  return r + "Date: Fri, 09 Oct 2026 05:36:12 UTC\r\nServer: Xmlrpc-c_Abyss/1.54.5\r\n\r\n" + body;
}

// <methodResponse> carrying one value, in the layout the controller uses.
inline std::string xmlResponse(const std::string & value_xml)
{
  return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<methodResponse>\r\n<params>\r\n<param>" +
         value_xml + "</param>\r\n</params>\r\n</methodResponse>\r\n";
}

// A valid state frame of `total` bytes: header, data_len, `fill` payload, correct checksum.
inline std::string makeFrame(std::size_t total, uint8_t fill = 0xAB, uint8_t cnt = 1)
{
  std::string f(total, static_cast<char>(fill));
  f[0] = 0x5A;
  f[1] = 0x5A;
  f[2] = static_cast<char>(cnt);
  const std::size_t len = total - fr_client_library::kFrameOverhead;
  f[3] = static_cast<char>(len & 0xFF);
  f[4] = static_cast<char>(len >> 8);
  uint16_t sum = 0;
  for (std::size_t i = 0; i < total - 2; ++i) {
    sum = static_cast<uint16_t>(sum + static_cast<uint8_t>(f[i]));
  }
  f[total - 2] = static_cast<char>(sum & 0xFF);
  f[total - 1] = static_cast<char>(sum >> 8);
  return f;
}

}  // namespace fr_test
