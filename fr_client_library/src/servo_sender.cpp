#include "fr_client_library/servo_sender.hpp"

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr_client_library
{
namespace
{
constexpr double kDeadBand = 0.5;   // entries around the target depth that are left alone
constexpr double kDamping = 0.8;

bool finite(const JointArray & a)
{
  for (const double v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

void raiseTo(std::atomic<uint64_t> & max, uint64_t v)
{
  uint64_t cur = max.load(std::memory_order_relaxed);
  while (v > cur && !max.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
  }
}
}  // namespace

ServoSender::ServoSender(std::string ip, uint16_t port, ServoSenderOptions options)
: options_(options), api_(std::move(ip), port, options.http)
{
}

ApiStatus ServoSender::start(const JointArray & initial_deg)
{
  if (options_.cmd_t_ms < 1 || options_.cmd_t_ms > 1000) {
    throw std::invalid_argument("cmdT must be a whole number of ms in [1, 1000]");
  }
  if (options_.prefill < 0 || options_.prefill > 64) {
    throw std::invalid_argument("prefill must be in [0, 64]");
  }
  if (!finite(initial_deg)) {
    throw std::invalid_argument("initial joint position must be finite");
  }
  if (options_.rt_priority < 0 || options_.rt_priority > 99) {
    throw std::invalid_argument("rt_priority must be in [0, 99]");
  }
  if (!(options_.loop_bandwidth_rad_s > 0)) {
    throw std::invalid_argument("loop bandwidth must be positive");
  }
  stop();

  if (!api_.connect()) {
    return ApiStatus{ApiError::Io, 0, "cannot connect"};
  }
  if (const ApiStatus st = api_.servoMoveStart(); !st.ok()) {
    api_.close();
    return st;
  }
  standing_ = initial_deg;
  Command drop;
  while (queue_.pop(drop)) {  // nothing is running: this thread is the consumer for now
  }
  comm_error_ = ApiError::None;
  halted_ = false;
  integral_ = 0;
  trim_ppm_ = 0;
  paused_ = false;
  have_last_sent_ = false;
  resetStream();
  stop_ = false;
  running_ = true;
  thread_ = std::thread(&ServoSender::run, this);
  rt_granted_ = false;
  if (options_.rt_priority > 0) {
    sched_param param{};
    param.sched_priority = options_.rt_priority;
    rt_granted_ = pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &param) == 0;
  }
  return {};
}

ApiStatus ServoSender::stop()
{
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  running_ = false;
  if (!api_.connected()) {
    return ApiStatus{ApiError::NotConnected, 0, {}};
  }
  const ApiStatus st = api_.servoMoveEnd();
  api_.close();
  return st;
}

bool ServoSender::enqueue(const JointArray & joint_deg)
{
  if (!running_.load(std::memory_order_relaxed) || !finite(joint_deg)) {
    return false;
  }
  if (!queue_.push(Command{joint_deg})) {
    overflows_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

ServoStats ServoSender::stats() const
{
  ServoStats s;
  s.sent = sent_;
  s.rejected = rejected_;
  s.suppressed = suppressed_;
  s.overflows = overflows_;
  s.max_backlog = max_backlog_;
  s.calls = calls_;
  s.max_call_us = max_call_us_;
  s.mean_call_us = s.calls == 0 ? 0.0 : static_cast<double>(call_us_sum_.load()) / s.calls;
  s.last_rejection = last_rejection_;
  s.trim_ppm = trim_ppm_;
  s.pauses = pauses_;
  return s;
}

bool ServoSender::send(const JointArray & q)
{
  const auto t0 = Clock::now();
  const ApiStatus st = api_.servoJ(q, options_.cmd_t_ms / 1000.0);
  const auto us = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count());
  ++calls_;
  call_us_sum_ += us;
  raiseTo(max_call_us_, us);

  if (st.ok()) {
    ++sent_;
    last_sent_ = q;
    have_last_sent_ = true;
  } else if (st.error == ApiError::Controller) {
    // The robot refused (e.g. a fault whose flag has not reached us yet): expected for the
    // frame in flight, not a communication failure. The caller decides from the state frame.
    ++rejected_;
    last_rejection_ = st.code;
  } else {
    comm_error_ = st.error;  // the API closed the connection; not retried (it may have arrived)
    return false;
  }
  return true;
}

void ServoSender::resetStream()
{
  window_.clear();
  frac_ = 0;
  primed_ = false;
  filter_valid_ = false;
  // The frames that follow the prefill still show the old depth: do not steer on them.
  const double settle_s = 0.3 / options_.loop_bandwidth_rad_s;
  settle_until_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(settle_s));
}

// PI loop on the firmware queue depth. The queue integrates (1 / cmdT) entries per second per unit
// of eps, so with bandwidth w: Kp = 2 zeta w cmdT, Ki = w^2 cmdT. The depth is low-passed (pole at
// 10 w) and a dead band of +-1 entry around the target leaves the trim alone while it is fine.
void ServoSender::updateTrim(Clock::time_point now)
{
  if (!options_.queue_depth || now < settle_until_) {
    return;
  }
  const std::optional<int> depth = options_.queue_depth();
  if (!depth || *depth < 0 || *depth > 255) {
    return;  // unknown or stale: keep the current trim
  }
  if (!filter_valid_) {
    filtered_ = *depth;
    filter_valid_ = true;
    last_update_ = now;
    return;
  }
  const double dt = std::chrono::duration<double>(now - last_update_).count();
  last_update_ = now;
  const double w = options_.loop_bandwidth_rad_s;
  const double cmd_t = options_.cmd_t_ms / 1000.0;
  filtered_ += (1.0 - std::exp(-10.0 * w * dt)) * (*depth - filtered_);

  const double error = filtered_ - options_.prefill;
  const double outside = std::abs(error) > kDeadBand ? error - std::copysign(kDeadBand, error) : 0.0;
  const double max_eps = kMaxTrimPpm * 1e-6;
  integral_ = std::clamp(integral_ + w * w * cmd_t * outside * dt, -max_eps, max_eps);
  const double eps = std::clamp(2.0 * kDamping * w * cmd_t * outside + integral_, -max_eps, max_eps);
  trim_ppm_ = eps * 1e6;
}

// The controller plays the first segment as soon as it arrives, so while the prefill is being sent
// (a ServoJ call takes about half a cmdT on a real controller) part of it is already gone. Keep
// sending standing segments until the estimated depth (sent minus played) reaches `prefill`; a
// controller that is slower than that gets at most 4 * prefill.
bool ServoSender::prefillStanding()
{
  const int limit = 4 * options_.prefill;
  const auto first = Clock::now();
  for (int sent = 0; sent < limit && !halted_ && !stop_; ++sent) {
    const double played = std::chrono::duration<double, std::milli>(Clock::now() - first).count() /
      options_.cmd_t_ms;
    if (sent >= options_.prefill && sent - played >= options_.prefill - 0.5) {
      break;
    }
    if (!send(standing_)) {
      return false;
    }
  }
  return true;
}

void ServoSender::run()
{
  try {
    stream();
  } catch (...) {
    comm_error_ = ApiError::Io;  // e.g. std::bad_alloc: handled like a broken connection
  }
  running_ = false;
}

void ServoSender::stream()
{
  using namespace std::chrono_literals;

  const Clock::duration update_every = std::chrono::milliseconds(options_.cmd_t_ms);
  Clock::time_point next_send{};
  Clock::time_point next_update{};
  bool starved = false;
  Command cmd;
  while (!stop_ && comm_error_.load() == ApiError::None) {
    if (halted_) {
      while (queue_.pop(cmd)) {
        ++suppressed_;
      }
      suppressed_ += window_.size();
      resetStream();
      paused_ = false;
      std::this_thread::sleep_for(1ms);
      continue;
    }

    if (paused_) {
      // Standing: drop the repeats; the first command that differs restarts the stream from the
      // standing position (prefill first), like start().
      bool moved = false;
      while (!moved && queue_.pop(cmd)) {
        moved = cmd.q != standing_;
      }
      if (!moved) {
        std::this_thread::sleep_for(1ms);
        continue;
      }
      resetStream();
      window_.push_back(cmd.q);
      paused_ = false;
      continue;
    }

    const auto now = Clock::now();
    if (now >= next_update) {
      updateTrim(now);
      next_update = now + update_every;
    }
    if (primed_ && now < next_send) {
      std::this_thread::sleep_until(std::min({next_send, next_update, now + 1ms}));
      continue;
    }

    // The window must hold command i, and i + 1 when the send falls between the two.
    const std::size_t need = primed_ ? (frac_ > 0 ? 2 : 1) : kLead;
    const std::size_t waiting = queue_.size();
    while (window_.size() < need && queue_.pop(cmd)) {
      window_.push_back(cmd.q);
    }
    if (window_.size() < need) {
      starved = true;  // the producer is late: wait, do not send a stale point
      std::this_thread::sleep_for(200us);
      continue;
    }
    if (!primed_) {
      // The prefill goes out only now, so that it is not played away while waiting for the lead.
      if (!prefillStanding()) {
        return;
      }
      if (!halted_ && !stop_) {
        primed_ = true;
        next_send = Clock::now();
      }
      continue;  // halt() may have arrived meanwhile
    }
    if (starved) {
      starved = false;
      next_send = now;  // no debt to catch up for time spent waiting for data
    }
    raiseTo(max_backlog_, waiting);

    JointArray target = window_[0];
    if (frac_ > 0) {
      for (int j = 0; j < 6; ++j) {
        target[j] += (window_[1][j] - window_[0][j]) * frac_;
      }
    }
    if (options_.at_rest && have_last_sent_ && target == last_sent_ && options_.at_rest(target)) {
      standing_ = target;   // the arm stands here: pause (sdk_quirks S18)
      paused_ = true;
      ++pauses_;
      continue;
    }
    if (!send(target)) {
      break;
    }
    const double eps = trim_ppm_.load() * 1e-6;
    next_send += std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double, std::milli>(options_.cmd_t_ms * (1.0 + eps)));
    frac_ += 1.0 + eps;
    while (frac_ >= 1.0 && !window_.empty()) {  // twice when eps > 0 and frac_ wraps
      frac_ -= 1.0;
      window_.pop_front();
    }
  }
}

}  // namespace fr_client_library
