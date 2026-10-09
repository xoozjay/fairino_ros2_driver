// Receives the state stream (TCP 20004) on its own thread and keeps the newest frame, which any
// thread may read. Reconnecting is the caller's decision: when the connection is lost the thread
// ends and running() turns false.
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <string>
#include <thread>

#include "fr_client_library/frame_parser.hpp"
#include "fr_client_library/tcp_socket.hpp"

namespace fr_client_library
{
struct StateSnapshot
{
  RobotState384 state{};
  std::chrono::steady_clock::time_point received_at{};
  uint64_t frame_count = 0;  // monotonic across restarts; compare with !=, 0 = nothing received

  // A frame is there and not older than max_age.
  bool fresh(std::chrono::steady_clock::duration max_age) const
  {
    return frame_count != 0 && std::chrono::steady_clock::now() - received_at <= max_age;
  }
};

struct StateReceiverOptions
{
  int connect_timeout_ms = 1000;
  int poll_ms = 50;  // how often the thread checks for stop() while the stream is quiet
};

class StateReceiver
{
public:
  explicit StateReceiver(std::string ip, uint16_t port = 20004, StateReceiverOptions options = {});
  ~StateReceiver() {stop();}
  StateReceiver(const StateReceiver &) = delete;
  StateReceiver & operator=(const StateReceiver &) = delete;

  // Connects (with timeout) and starts the thread, expecting the frames of `version` (throws
  // std::invalid_argument if their size is unknown). Restarts if already running; frames from
  // before are not reported any more. start() and stop() must not run concurrently; every other
  // member may be called from any thread meanwhile.
  bool start(const SoftwareVersion & version);
  void stop();
  bool running() const {return running_.load();}

  // Any thread: the newest frame. False until a frame has arrived since start(). Waits while the
  // receiver thread copies a frame in (about a microsecond, once per frame).
  bool latest(StateSnapshot & out) const;
  // The same, but never waits: false and `out` left alone when the receiver thread holds the frame
  // this instant. For realtime readers, which keep their previous frame then; its age still tells
  // when it is too old, and the next frame period gives another chance.
  bool tryLatest(StateSnapshot & out) const;

  // Non-realtime: true once a frame newer than at start() has arrived.
  bool waitForFrame(int timeout_ms);

  FrameParser::Stats stats() const;

private:
  void run();
  void receive();

  std::string ip_;
  uint16_t port_;
  SoftwareVersion version_ = kTargetVersion;
  StateReceiverOptions options_;

  TcpSocket sock_;
  std::thread thread_;
  mutable std::mutex newest_mutex_;
  StateSnapshot newest_;  // guarded by newest_mutex_
  uint64_t count_ = 0;  // producer-side frame counter
  uint64_t count_at_start_ = 0;

  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> frames_{0};
  std::atomic<uint64_t> bad_frames_{0};
  std::atomic<uint64_t> wrong_size_frames_{0};
  std::atomic<std::size_t> last_wrong_size_{0};
};

}  // namespace fr_client_library
