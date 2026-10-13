// Sends MoveGripper from its own thread and its own XML-RPC connection (TCP 20003), so that the
// realtime side never waits for the network and the servo stream's connection is not shared.
//
// Only the newest command counts: post() overwrites one that has not gone out yet. A call takes
// about 20-40 ms on a real controller and does not wait for the motion (block = 1, sdk_quirks G6;
// block = 0 does not block either and faults the gripper, G11). While a ServoJ stream flows the
// controller answers only once it stops (G8): post only while the stream is paused.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "fr_client_library/robot_api.hpp"

namespace fr_client_library
{
struct GripperCommand
{
  int position = 0;  // percent, 100 = fully open
  int velocity = 0;  // percent
  int force = 0;     // percent
};

struct GripperSenderStats
{
  uint64_t sent = 0;     // MoveGripper calls the controller accepted
  uint64_t failed = 0;   // calls that failed (transport, refusal, error code)
  ApiStatus last_failure;
};

class GripperSender
{
public:
  static constexpr int kMaxTimeMs = 30000;  // MoveGripper's max_time: the largest the SDK allows

  GripperSender(std::string ip, uint16_t port, int index, HttpOptions http = {1000, 1000});
  ~GripperSender() {stop();}
  GripperSender(const GripperSender &) = delete;
  GripperSender & operator=(const GripperSender &) = delete;

  // Checks that the controller accepts a connection, then starts the thread. False if not.
  bool start();
  // Drops a command that has not gone out yet and joins the thread (waits for a call in progress).
  void stop();
  bool running() const {return running_.load();}

  // Realtime side, never blocks: replaces the pending command. False if the slot is busy this
  // instant or the sender is not running; try again next cycle.
  bool post(const GripperCommand & command);
  // Realtime side, never blocks: the pending command (if any) is not sent.
  void discard() {discard_.store(true);}

  GripperSenderStats stats() const;

private:
  void run();
  void serve();
  void fail(const ApiStatus & st);

  RobotApi api_;
  int index_;
  std::thread thread_;
  std::mutex mutex_;  // guards pending_ and stop_flag_
  std::condition_variable cv_;
  std::optional<GripperCommand> pending_;
  bool stop_flag_ = false;
  std::atomic<bool> discard_{false};
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> sent_{0};
  std::atomic<uint64_t> failed_{0};
  mutable std::mutex stats_mutex_;   // guards last_failure_
  ApiStatus last_failure_;
};

}  // namespace fr_client_library
