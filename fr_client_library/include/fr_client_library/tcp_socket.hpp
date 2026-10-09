// Minimal non-blocking IPv4 TCP client: every operation has a timeout.
// Not thread-safe; one thread uses a given instance.
#pragma once

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace fr_client_library
{
class TcpSocket
{
public:
  TcpSocket() = default;
  ~TcpSocket() { close(); }
  TcpSocket(const TcpSocket &) = delete;
  TcpSocket & operator=(const TcpSocket &) = delete;

  bool connect(const std::string & ip, uint16_t port, int timeout_ms);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // True if all bytes were sent before the timeout.
  bool sendAll(const char * data, std::size_t size, int timeout_ms);

  // >0: bytes read, 0: nothing within timeout, -1: peer closed or error.
  ssize_t recvSome(char * buf, std::size_t size, int timeout_ms);

private:
  int fd_ = -1;
};

}  // namespace fr_client_library
