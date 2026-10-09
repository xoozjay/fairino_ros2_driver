// HTTP/1.1 long-lived connection for XML-RPC POSTs (TCP 20003).
//
// What the controller's server does (simulator, docs/sdk_quirks.md C7): keeps the connection open
// ("Keep-Alive: timeout=15, max=30" -- max is static, not a countdown), answers the 30th request
// with "Connection: close" and drops it, and drops idle connections after 15 s.
//
// Rotation therefore happens only *between* requests, where nothing is in flight: a response with
// "Connection: close", or a connection idle for nearly the announced timeout, is replaced by a
// fresh one before the next post(). Any error mid-request closes the connection and is reported;
// there is no retry, because a request that may have been delivered must not be repeated.
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "fr_client_library/tcp_socket.hpp"

namespace fr_client_library
{
enum class HttpStatus { Ok, NotConnected, Timeout, IoError, ProtocolError };

struct HttpResult
{
  HttpStatus status = HttpStatus::NotConnected;
  std::string body;  // valid when status == Ok
};

struct HttpOptions
{
  int connect_timeout_ms = 1000;
  int request_timeout_ms = 1000;  // sending + waiting for the complete response
};

class HttpConnection
{
public:
  HttpConnection(std::string ip, uint16_t port = 20003, HttpOptions options = {});

  bool connect();
  void close();
  bool isOpen() const {return sock_.isOpen();}

  // POST /RPC2. Not thread-safe; one request in flight.
  HttpResult post(const std::string & body);

private:
  HttpResult fail(HttpStatus status);
  bool idleTooLong() const;

  std::string ip_;
  uint16_t port_;
  HttpOptions options_;
  TcpSocket sock_;
  bool rotate_ = false;  // closed on purpose between requests: reconnect before the next post
  int keepalive_timeout_ms_ = 0;  // announced by the server, 0 = unknown
  std::chrono::steady_clock::time_point last_activity_{};
};

}  // namespace fr_client_library
