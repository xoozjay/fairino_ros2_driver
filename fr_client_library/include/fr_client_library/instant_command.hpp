// Instant-command channel (TCP 8080) of software version 3.8.4.
//
// Frames follow fairino-cpp-sdk v2.2.4-robot3.8.4 byte for byte (including the
// fixed counter 44 of STOP); see docs/sdk_quirks.md L7-L10. Only STOP exists: the
// SDK's other commands on this channel (PAUSE, RESUME, StopExtAxisJog,
// ConveyorComDetectTrigger) are not used by the driver. Unlike the SDK, send() is
// synchronous: it waits for the reply, and any failure closes the connection so
// that a command can never sit in the controller's backlog and run later.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "fr_client_library/tcp_socket.hpp"

namespace fr_client_library
{
// Values are the command ids on the wire.
enum class InstantCommand : int {
  Stop = 102,
};

// "/f/bIII<cnt>III<id>III<len>III<content>III/b/f"; STOP's counter is always 44.
std::string encodeInstantCommand(InstantCommand cmd);

struct InstantReply
{
  int cnt = 0;
  int cmd_id = 0;
  std::string content;  // "1" for STOP on the simulator; meaning unconfirmed, not interpreted
};

// Reassembles replies from an arbitrary TCP byte stream; resynchronises on garbage.
class InstantReplyParser
{
public:
  void feed(const char * data, std::size_t size, std::vector<InstantReply> & out);

private:
  std::string buf_;
};

enum class InstantStatus { Ok, NotConnected, Timeout, IoError };

struct InstantResult
{
  InstantStatus status = InstantStatus::NotConnected;
  std::string content;  // reply content, valid when status == Ok
};

// One command in flight; not thread-safe. Single user of 8080 on the controller.
class InstantCommandChannel
{
public:
  struct Options
  {
    int connect_timeout_ms = 1000;
    int reply_timeout_ms = 1000;  // covers sending and waiting for the matching reply
  };

  InstantCommandChannel(std::string ip, uint16_t port, Options options);
  InstantCommandChannel(std::string ip, uint16_t port = 8080)
  : InstantCommandChannel(std::move(ip), port, Options{}) {}

  bool connect();
  void disconnect() { sock_.close(); }
  bool connected() const { return sock_.isOpen(); }

  // Replies are matched by command id. Timeout / IoError close the connection;
  // reconnecting is the caller's decision.
  InstantResult send(InstantCommand cmd);

private:
  InstantResult fail(InstantStatus status);

  std::string ip_;
  uint16_t port_;
  Options options_;
  TcpSocket sock_;
  InstantReplyParser parser_;
};

}  // namespace fr_client_library
