#include "fr_client_library/gripper_sender.hpp"

#include <exception>
#include <string>

namespace fr_client_library
{
GripperSender::GripperSender(std::string ip, uint16_t port, int index, HttpOptions http)
: api_(std::move(ip), port, http), index_(index)
{
}

bool GripperSender::start()
{
  stop();
  // Only a reachability check: the controller drops a connection idle for 15 s, and the first
  // command may come much later, so the thread connects when it has something to send.
  if (!api_.connect()) {
    return false;
  }
  api_.close();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    stop_flag_ = false;
  }
  discard_ = false;
  running_ = true;
  thread_ = std::thread(&GripperSender::run, this);
  return true;
}

void GripperSender::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_flag_ = true;
    pending_.reset();
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  running_ = false;
  api_.close();
}

bool GripperSender::post(const GripperCommand & command)
{
  if (!running_.load()) {
    return false;
  }
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  pending_ = command;
  discard_ = false;  // a discard() before this post() is about an older command
  lock.unlock();
  cv_.notify_one();
  return true;
}

GripperSenderStats GripperSender::stats() const
{
  GripperSenderStats s;
  s.sent = sent_;
  s.failed = failed_;
  std::lock_guard<std::mutex> lock(stats_mutex_);
  s.last_failure = last_failure_;
  return s;
}

void GripperSender::run()
{
  try {
    serve();
  } catch (const std::exception & e) {
    fail(ApiStatus{ApiError::Io, 0, std::string("gripper sender thread: ") + e.what()});
  } catch (...) {
    fail(ApiStatus{ApiError::Io, 0, "gripper sender thread: unknown exception"});
  }
  running_ = false;   // post() refuses from now on
}

void GripperSender::fail(const ApiStatus & st)
{
  {
    std::lock_guard<std::mutex> guard(stats_mutex_);
    last_failure_ = st;
  }
  ++failed_;   // after last_failure_: whoever sees the count can read the reason
}

void GripperSender::serve()
{
  std::unique_lock<std::mutex> lock(mutex_);
  for (;;) {
    cv_.wait(lock, [this] {return stop_flag_ || pending_.has_value();});
    if (stop_flag_) {
      return;
    }
    if (discard_.exchange(false)) {
      pending_.reset();
      continue;
    }
    const GripperCommand c = *pending_;
    pending_.reset();
    lock.unlock();
    if (!api_.connected()) {
      api_.connect();  // first command, or the previous call broke the connection
    }
    const ApiStatus st = api_.moveGripper(index_, c.position, c.velocity, c.force, kMaxTimeMs);
    if (st.ok()) {
      ++sent_;
    } else {
      fail(st);
    }
    lock.lock();
  }
}

}  // namespace fr_client_library
