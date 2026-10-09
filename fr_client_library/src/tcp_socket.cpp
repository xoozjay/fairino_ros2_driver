#include "fr_client_library/tcp_socket.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>

namespace fr_client_library
{
namespace
{
using Clock = std::chrono::steady_clock;

bool waitFor(int fd, short events, int timeout_ms)
{
  pollfd p{fd, events, 0};
  for (;;) {
    const int r = ::poll(&p, 1, timeout_ms);
    if (r < 0 && errno == EINTR) {
      continue;  // timeout restarts; the drift is irrelevant at our scales
    }
    return r > 0;
  }
}

int remainingMs(Clock::time_point deadline)
{
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
  return left.count() > 0 ? static_cast<int>(left.count()) : 0;
}
}  // namespace

bool TcpSocket::connect(const std::string & ip, uint16_t port, int timeout_ms)
{
  close();
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    return false;
  }
  fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    return false;
  }
  const int one = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  if (::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    if (errno != EINPROGRESS || !waitFor(fd_, POLLOUT, timeout_ms)) {
      close();
      return false;
    }
    int err = 0;
    socklen_t len = sizeof(err);
    if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0) {
      close();
      return false;
    }
  }
  return true;
}

void TcpSocket::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool TcpSocket::sendAll(const char * data, std::size_t size, int timeout_ms)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (size > 0) {
    const ssize_t n = ::send(fd_, data, size, MSG_NOSIGNAL);
    if (n > 0) {
      data += n;
      size -= static_cast<std::size_t>(n);
    } else if (n < 0 && errno == EINTR) {
      continue;
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (!waitFor(fd_, POLLOUT, remainingMs(deadline))) {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

ssize_t TcpSocket::recvSome(char * buf, std::size_t size, int timeout_ms)
{
  if (!waitFor(fd_, POLLIN, timeout_ms)) {
    return 0;
  }
  const ssize_t n = ::recv(fd_, buf, size, 0);
  if (n > 0) {
    return n;
  }
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
    return 0;
  }
  return -1;  // EOF or error
}

}  // namespace fr_client_library
