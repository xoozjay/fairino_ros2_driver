#include "fr_client_library/state_receiver.hpp"

namespace fr_client_library
{
StateReceiver::StateReceiver(std::string ip, uint16_t port, StateReceiverOptions options)
: ip_(std::move(ip)), port_(port), options_(options)
{
}

bool StateReceiver::start(const SoftwareVersion & version)
{
  stop();
  FrameParser{version};  // fail early (throws) for a version without a known frame size
  version_ = version;
  {
    std::lock_guard<std::mutex> lock(newest_mutex_);
    newest_ = StateSnapshot{};
  }
  if (!sock_.connect(ip_, port_, options_.connect_timeout_ms)) {
    return false;
  }
  stop_ = false;
  count_at_start_ = frames_.load();
  running_ = true;
  thread_ = std::thread(&StateReceiver::run, this);
  return true;
}

void StateReceiver::stop()
{
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  sock_.close();
  running_ = false;
}

void StateReceiver::run()
{
  try {
    receive();
  } catch (...) {
    // e.g. std::bad_alloc: the same as a lost connection, which the consumer handles
  }
  running_ = false;
}

void StateReceiver::receive()
{
  FrameParser parser(version_);
  std::vector<RobotState384> frames;
  char buf[2048];

  while (!stop_) {
    const ssize_t n = sock_.recvSome(buf, sizeof(buf), options_.poll_ms);
    if (n < 0) {
      break;  // connection lost
    }
    if (n == 0) {
      continue;
    }
    frames.clear();
    parser.feed(buf, static_cast<std::size_t>(n), frames);

    const auto & s = parser.stats();
    bad_frames_ = s.bad_frames;
    wrong_size_frames_ = s.wrong_size_frames;
    last_wrong_size_ = s.last_wrong_size;

    if (!frames.empty()) {
      count_ += frames.size();
      StateSnapshot snap;
      snap.state = frames.back();  // older frames in the same chunk are obsolete
      snap.received_at = std::chrono::steady_clock::now();
      snap.frame_count = count_;
      {
        std::lock_guard<std::mutex> lock(newest_mutex_);
        newest_ = snap;
      }
      frames_ = count_;
    }
  }
}

bool StateReceiver::latest(StateSnapshot & out) const
{
  std::lock_guard<std::mutex> lock(newest_mutex_);
  out = newest_;
  return out.frame_count != 0;
}

bool StateReceiver::tryLatest(StateSnapshot & out) const
{
  std::unique_lock<std::mutex> lock(newest_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  out = newest_;
  return out.frame_count != 0;
}

bool StateReceiver::waitForFrame(int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (frames_.load() == count_at_start_) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

FrameParser::Stats StateReceiver::stats() const
{
  FrameParser::Stats s;
  s.bad_frames = bad_frames_;
  s.wrong_size_frames = wrong_size_frames_;
  s.last_wrong_size = last_wrong_size_;
  return s;
}

}  // namespace fr_client_library
