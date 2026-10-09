// Streams ServoJ commands to the controller from its own (non-realtime) thread.
//
// How the firmware plays ServoJ (docs/sdk_quirks.md S1, S14-S17): every call is one straight
// segment of cmdT between consecutive targets, queued in a 255-entry buffer and played back at
// 1 kHz by the controller's own clock. Consequences that shape this class:
//
//  * The firmware queue is the jitter buffer. Just before the first command goes out (and again
//    after an idle pause) the sender fills it with `prefill` segments that stand still at the initial
//    position, so a late command finds the robot still busy. The price is a
//    constant latency of prefill * cmdT between enqueue() and motion.
//  * The controller's clock differs from the host's by about 1000 ppm and the difference varies
//    (S17). Sending one segment per command would make the queue grow (or drain) steadily, so the
//    command stream is resampled: the sender runs on its own schedule with period
//    cmdT * (1 + eps) and sends the point of the command polyline at the matching position (linear
//    interpolation, advancing 1 + eps commands per send). eps is trimmed by a PI loop on the
//    firmware queue depth (options.queue_depth), limited to +-kMaxTrimPpm (1%). The path keeps its
//    shape and is only stretched by eps; no point is dropped or repeated, so there are no
//    speed spikes. The loop needs no new command: it reads the depth the state frames carry.
//  * Commands are assumed to be produced at the nominal rate 1 / cmdT of the host clock (the host
//    backlog then stays constant). The first send waits until kLead commands are queued: one
//    interval of margin so that a late command does not stall the interpolation.
//  * If the sender falls behind it catches up; the firmware queue absorbs the burst.
//  * Idle pause (options.at_rest): while the arm stands, nothing is sent (sdk_quirks S18). As soon
//    as the point to send is bit for bit the one sent last and the state frames show the arm there,
//    the stream pauses (no ServoMoveEnd); the queue drains and the robot holds. The first command
//    that differs is sent like a fresh start: prefill at the standing position, then the stream.
//    The controller holds back gripper commands while ServoJ flows (G8), so they need the pause.
//
// Stopping the stream does not stop the robot: what is queued still runs. Use STOP (8080).
//
// The sender owns its own XML-RPC connection, so ServoMoveStart/ServoJ/ServoMoveEnd are strictly
// ordered on one connection. Use a separate RobotApi for everything else (e.g. reset_error).
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <thread>

#include "fr_client_library/robot_api.hpp"
#include "fr_client_library/spsc_queue.hpp"

namespace fr_client_library
{
// Current firmware queue depth (state frame mc_queue_len); nullopt when unknown or stale. Called
// from the sender thread a few times per cmdT, so it must be thread-safe and cheap.
using QueueDepthFn = std::function<std::optional<int>()>;
// Do fresh state frames show the arm at this position? Same calling rules as QueueDepthFn.
using AtRestFn = std::function<bool(const JointArray & target_deg)>;

struct ServoSenderOptions
{
  int cmd_t_ms = 8;  // ServoJ's cmdT: whole ms in [1, 1000]; sdk_quirks S2: start at 125 Hz
  int prefill = 3;   // standing segments before the first command and the target queue depth, in [0, 64]
  // The request timeout is the longest a stuck call can keep the stream silent. ❓ to be chosen
  // after measuring ServoJ on real hardware (ServoStats has the numbers).
  HttpOptions http{1000, 100};
  // Feedback for the drift compensation. Empty: no compensation (eps stays 0). The target depth is
  // `prefill`, with a dead band of +-0.5.
  QueueDepthFn queue_depth;
  // Bandwidth of the depth loop. The queue is an integrator (1 / cmdT entries per second per unit
  // eps), so this alone fixes the gains. 0.1 rad/s settles in about 40 s: slow enough to ignore
  // jitter in the depth, fast enough for a drift that changes with temperature.
  double loop_bandwidth_rad_s = 0.1;
  // SCHED_FIFO priority (1..99) for the sender thread; 0 leaves the scheduler alone. Needs
  // CAP_SYS_NICE or an rtprio limit; if refused the sender runs normally (realtimeGranted()).
  // Not required: the firmware queue absorbs scheduling jitter up to about prefill * cmdT.
  int rt_priority = 0;
  // Idle pause (see above). Empty: the stream never pauses.
  AtRestFn at_rest;
};

struct ServoStats
{
  uint64_t sent = 0;        // ServoJ calls that the controller accepted (prefill included)
  uint64_t rejected = 0;    // calls the controller answered with an error code
  uint64_t suppressed = 0;  // commands discarded while halted
  uint64_t overflows = 0;   // enqueue() refused because the host queue was full
  uint64_t max_backlog = 0; // most commands ever waiting on the host
  uint64_t calls = 0;       // ServoJ calls made
  double mean_call_us = 0;  // mean duration of a ServoJ call
  uint64_t max_call_us = 0;
  int last_rejection = 0;   // error code of the most recent rejection
  double trim_ppm = 0;      // current send period correction eps (positive: sending slower)
  uint64_t pauses = 0;      // idle pauses so far
};

class ServoSender
{
public:
  static constexpr std::size_t kQueueCapacity = 64;  // host side; 512 ms at 8 ms
  static constexpr double kMaxTrimPpm = 10000;
  static constexpr std::size_t kLead = 3;  // commands queued before the first send

  ServoSender(std::string ip, uint16_t port = 20003, ServoSenderOptions options = {});
  ~ServoSender() {stop();}
  ServoSender(const ServoSender &) = delete;
  ServoSender & operator=(const ServoSender &) = delete;

  // Connects, calls ServoMoveStart and starts the thread, which sends the prefill once commands
  // are queued (see kLead).
  // `initial_deg` must be the robot's *actual* joint position (from a validated state frame,
  // never zeros: docs R3). Throws std::invalid_argument for bad options or non-finite input.
  ApiStatus start(const JointArray & initial_deg);

  // Stops the thread (commands still waiting on the host are dropped), then calls ServoMoveEnd.
  // Neither empties the firmware queue and neither means "the robot stopped".
  ApiStatus stop();

  bool running() const {return running_.load();}
  // True if options.rt_priority was requested and the scheduler accepted it (valid after start()).
  bool realtimeGranted() const {return rt_granted_.load();}
  // None while healthy; the transport error that ended the thread otherwise.
  ApiError commError() const {return comm_error_.load();}

  // Realtime side, wait-free: the next segment. False (not queued) if the sender is not running,
  // the value is non-finite, or the host queue is full (counted in stats().overflows; treat as a
  // communication problem).
  bool enqueue(const JointArray & joint_deg);

  // Paused because the arm stands (options.at_rest); nothing is being sent.
  bool paused() const {return paused_.load();}

  // ROBOT_FAULT / ESTOP: send nothing and discard what is queued on the host, until the next
  // start() (after the operator has cleared the fault: a fresh session and prefill).
  void halt() {halted_.store(true);}

  ServoStats stats() const;

private:
  using Clock = std::chrono::steady_clock;

  struct Command
  {
    JointArray q{};
  };

  void run();
  void stream();
  bool send(const JointArray & q);  // false: transport error, the thread must end
  bool prefillStanding();  // false: transport error
  void resetStream();
  void updateTrim(Clock::time_point now);

  ServoSenderOptions options_;
  RobotApi api_;
  std::thread thread_;
  SpscQueue<Command, kQueueCapacity> queue_;
  JointArray standing_{};

  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> halted_{false};
  std::atomic<ApiError> comm_error_{ApiError::None};
  std::atomic<uint64_t> sent_{0};
  std::atomic<uint64_t> rejected_{0};
  std::atomic<uint64_t> suppressed_{0};
  std::atomic<uint64_t> overflows_{0};
  std::atomic<uint64_t> max_backlog_{0};
  std::atomic<uint64_t> calls_{0};
  std::atomic<uint64_t> call_us_sum_{0};
  std::atomic<uint64_t> max_call_us_{0};
  std::atomic<int> last_rejection_{0};
  std::atomic<double> trim_ppm_{0};
  std::atomic<bool> rt_granted_{false};
  std::atomic<bool> paused_{false};
  std::atomic<uint64_t> pauses_{0};

  // Sender thread only. window_ holds commands i, i+1, ...; the next send is at i + frac_.
  std::deque<JointArray> window_;
  double frac_ = 0;
  bool primed_ = false;
  JointArray last_sent_{};
  bool have_last_sent_ = false;
  double integral_ = 0;       // trim, in units of eps
  double filtered_ = 0;       // low-passed queue depth
  bool filter_valid_ = false;
  Clock::time_point settle_until_{};
  Clock::time_point last_update_{};
};

}  // namespace fr_client_library
