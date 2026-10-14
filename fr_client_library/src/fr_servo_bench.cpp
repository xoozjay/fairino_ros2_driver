// Servo streaming benchmark: how deep does the firmware's ServoJ queue (state frame mc_queue_len)
// stay while we stream a slow smooth wave on one joint under configurable jitter and prefill, with
// ServoSender's drift compensation on (--no-compensation turns it off)? See docs/sdk_quirks.md
// S15-S17.
//
// THIS MOVES THE ROBOT: joint N swings by --amplitude-deg (default 0.5, at most 30) in a raised
// cosine that starts and ends at rest at the starting position. It refuses to run without --move.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "fr_client_library/robot_api.hpp"
#include "fr_client_library/servo_sender.hpp"
#include "fr_client_library/state_receiver.hpp"

using namespace fr_client_library;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace
{
struct Args
{
  std::string ip;
  bool move = false;
  bool allow_unverified = false;
  double seconds = 10;
  int cmd_t_ms = 8;
  int prefill = 3;
  double jitter_ms = 0;     // each enqueue is delayed by uniform [0, jitter] after its scheduled time
  bool compensate = true;
  double amplitude_deg = 0.5;
  double cycle_s = 2.0;
  int joint = 1;
};

void usage(const char * argv0)
{
  std::fprintf(stderr,
    "usage: %s <ip> --move [options]\n"
    "  Streams a slow wave on one joint and reports the firmware queue depth (mc_queue_len).\n"
    "  --move               required: confirm that the robot may move\n"
    "  --seconds S          duration, rounded up to whole wave cycles   (default 10)\n"
    "  --cmd-t-ms N         ServoJ cmdT / producer period in whole ms   (default 8)\n"
    "  --prefill N          standing segments sent at start             (default 3)\n"
    "  --jitter-ms X        random delay 0..X ms on every enqueue       (default 0)\n"
    "  --no-compensation    leave the clock drift between host and controller uncorrected\n"
    "  --amplitude-deg A    wave amplitude, at most 30; 0 = stand still (default 0.5)\n"
    "  --cycle-s S          wave cycle length                           (default 2)\n"
    "  --joint N            joint 1..6                                  (default 1)\n"
    "  --allow-unverified   run even if the software version is not whitelisted\n",
    argv0);
}

bool parse(int argc, char ** argv, Args & a)
{
  if (argc < 2) {return false;}
  a.ip = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string k = argv[i];
    const auto num = [&](double & out) {
        if (i + 1 >= argc) {return false;}
        out = std::atof(argv[++i]);
        return true;
      };
    double v = 0;
    if (k == "--move") {a.move = true;}
    else if (k == "--allow-unverified") {a.allow_unverified = true;}
    else if (k == "--seconds" && num(a.seconds)) {}
    else if (k == "--jitter-ms" && num(a.jitter_ms)) {}
    else if (k == "--no-compensation") {a.compensate = false;}
    else if (k == "--amplitude-deg" && num(a.amplitude_deg)) {}
    else if (k == "--cycle-s" && num(a.cycle_s)) {}
    else if (k == "--cmd-t-ms" && num(v)) {a.cmd_t_ms = static_cast<int>(v);}
    else if (k == "--prefill" && num(v)) {a.prefill = static_cast<int>(v);}
    else if (k == "--joint" && num(v)) {a.joint = static_cast<int>(v);}
    else {std::fprintf(stderr, "unknown or incomplete option: %s\n", k.c_str()); return false;}
  }
  if (a.seconds <= 0 || a.cycle_s <= 0 || a.jitter_ms < 0 || a.amplitude_deg < 0 ||
    a.amplitude_deg > 30.0 || a.joint < 1 || a.joint > 6 || a.cmd_t_ms < 1 || a.cmd_t_ms > 1000 ||
    a.prefill < 0 || a.prefill > 64)
  {
    std::fprintf(stderr, "option out of range\n");
    return false;
  }
  return true;
}

struct Sample
{
  double t;  // seconds since the stream started
  int queue;
  unsigned robot_state;
  double pos;
};

struct CmdPoint
{
  double t;
  double pos;
};
}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  if (!parse(argc, argv, a)) {
    usage(argv[0]);
    return 2;
  }
  if (!a.move) {
    std::fprintf(stderr, "refusing to run: this moves joint %d by +-%.2f deg. Add --move.\n",
      a.joint, a.amplitude_deg);
    return 2;
  }

  // ---- version and preconditions ----
  RobotApi api(a.ip);
  if (!api.connect()) {std::fprintf(stderr, "cannot connect to %s:20003\n", a.ip.c_str()); return 1;}
  const auto info = api.getSoftwareVersion();
  if (!info.ok() || !info.value.version) {
    std::fprintf(stderr, "cannot determine the software version\n");
    return 1;
  }
  const SoftwareVersion version = *info.value.version;
  std::printf("software %s (%s)\n", info.value.software_version.c_str(),
    isVerifiedVersion(version) ? "verified" : "NOT verified");
  if (!isVerifiedVersion(version) && !a.allow_unverified) {
    std::fprintf(stderr, "software version is not whitelisted; --allow-unverified to override\n");
    return 1;
  }
  if (stateFrameSize(version) == 0) {
    std::fprintf(stderr, "no known state frame size for this version\n");
    return 1;
  }

  StateReceiver receiver(a.ip);
  if (!receiver.start(version) || !receiver.waitForFrame(2000)) {
    std::fprintf(stderr, "no state frames\n");
    return 1;
  }
  StateSnapshot snap;
  std::this_thread::sleep_for(50ms);
  receiver.latest(snap);
  if (snap.state.main_code != 0 || snap.state.EmergencyStop != 0 || snap.state.rbtEnableState != 1) {
    std::fprintf(stderr, "robot not ready: main_code=%d estop=%u enable=%d\n", snap.state.main_code,
      snap.state.EmergencyStop, snap.state.rbtEnableState);
    return 1;
  }
  JointArray home;
  for (int i = 0; i < 6; ++i) {home[i] = snap.state.jt_cur_pos[i];}
  const int jx = a.joint - 1;

  // ---- plan ----
  const double peak_step_deg = a.amplitude_deg * 0.5 * 6.283185307179586 / a.cycle_s * a.cmd_t_ms / 1000.0;
  // the firmware faults on large jumps (sdk_quirks S11); 1.5 deg admits 180 deg/s at 8 ms (1.44 deg)
  if (peak_step_deg > 1.5) {
    std::fprintf(stderr, "peak step %.2f deg per command is too large; lengthen --cycle-s\n", peak_step_deg);
    return 2;
  }
  const int cycle_ticks = std::max(2, static_cast<int>(std::lround(a.cycle_s * 1000.0 / a.cmd_t_ms)));
  const int cycles = std::max(1, static_cast<int>(std::ceil(a.seconds * 1000.0 / a.cmd_t_ms / cycle_ticks)));
  const long ticks = static_cast<long>(cycles) * cycle_ticks;
  const double two_pi = 6.283185307179586;
  const auto plan = [&](long k) {
      JointArray t = home;
      t[jx] = home[jx] + a.amplitude_deg * 0.5 * (1.0 - std::cos(two_pi * k / cycle_ticks));
      return t;
    };
  std::printf("joint %d wave +-%.2f deg (max %.2f deg/s), %ld ticks of %d ms (%d cycles), prefill %d, "
    "jitter 0..%.1f ms, drift compensation %s\n", a.joint, a.amplitude_deg,
    a.amplitude_deg * 0.5 * two_pi / a.cycle_s, ticks, a.cmd_t_ms, cycles, a.prefill, a.jitter_ms,
    a.compensate ? "on" : "off");

  // ---- run ----
  ServoSenderOptions opt;
  opt.cmd_t_ms = a.cmd_t_ms;
  opt.prefill = a.prefill;
  if (a.compensate) {
    opt.queue_depth = [&receiver]() -> std::optional<int> {
        StateSnapshot s;
        if (!receiver.latest(s)) {return std::nullopt;}
        return s.state.mc_queue_len;
      };
  }
  ServoSender sender(a.ip, 20003, opt);
  const auto started = sender.start(home);
  if (!started.ok()) {
    std::fprintf(stderr, "ServoMoveStart failed: error %d code %d\n", static_cast<int>(started.error), started.code);
    return 1;
  }

  const auto t0 = Clock::now();
  std::atomic<bool> monitor_run{true};
  std::vector<Sample> samples;
  samples.reserve(static_cast<std::size_t>(ticks) + 1000);
  std::thread monitor([&] {
      uint64_t last = 0;
      StateSnapshot s;
      while (monitor_run) {
        if (receiver.latest(s) && s.frame_count != last) {
          last = s.frame_count;
          samples.push_back({std::chrono::duration<double>(Clock::now() - t0).count(),
              s.state.mc_queue_len, s.state.robot_state, s.state.jt_cur_pos[jx]});
        }
        std::this_thread::sleep_for(500us);
      }
    });

  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> jitter(0.0, a.jitter_ms);
  long refused = 0;
  std::vector<CmdPoint> cmd_points;
  cmd_points.reserve(static_cast<std::size_t>(ticks) + 10);
  const auto ms = [](double v) {return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(v));};
  for (long k = 0; k <= ticks; ++k) {   // k == ticks is the final point, back at rest at home
    std::this_thread::sleep_until(t0 + ms(a.cmd_t_ms * k + (a.jitter_ms > 0 ? jitter(rng) : 0.0)));
    const auto p = plan(k);
    cmd_points.push_back({std::chrono::duration<double>(Clock::now() - t0).count(), p[jx]});
    if (!sender.enqueue(p)) {++refused;}
  }
  const double produced_s = std::chrono::duration<double>(Clock::now() - t0).count();
  const double trim_ppm = sender.stats().trim_ppm;  // before the drain, which pulls it away

  // let the firmware queue drain
  const auto drain_deadline = Clock::now() + std::chrono::milliseconds(255 * a.cmd_t_ms + 1000);
  while (Clock::now() < drain_deadline && !samples.empty() && samples.back().queue > 0) {
    std::this_thread::sleep_for(10ms);
  }
  std::this_thread::sleep_for(200ms);
  monitor_run = false;
  monitor.join();
  const auto ended = sender.stop();
  receiver.latest(snap);
  const double final_err = snap.state.jt_cur_pos[jx] - home[jx];
  const ServoStats st = sender.stats();

  // ---- report (samples while streaming, without the first 0.3 s of start-up) ----
  std::vector<Sample> run;
  for (const auto & s : samples) {
    if (s.t >= 0.3 && s.t <= produced_s) {run.push_back(s);}
  }
  if (run.size() < 10) {std::fprintf(stderr, "too few state samples (%zu)\n", run.size()); return 1;}
  double sum = 0, mn = 1e9, mx = 0;
  std::map<int, int> hist;
  int stops = 0;
  unsigned prev = 2;
  for (const auto & s : run) {
    sum += s.queue; mn = std::min<double>(mn, s.queue); mx = std::max<double>(mx, s.queue);
    ++hist[s.queue];
    if (s.robot_state == 1 && prev != 1) {++stops;}
    prev = s.robot_state;
  }
  const double mean = sum / run.size();
  double tm = 0;
  for (const auto & s : run) {tm += s.t;}
  tm /= run.size();
  double num = 0, den = 0;
  for (const auto & s : run) {num += (s.t - tm) * (s.queue - mean); den += (s.t - tm) * (s.t - tm);}
  const double slope = den > 0 ? num / den : 0;  // entries per second

  std::printf("\nfirmware queue (mc_queue_len, %zu samples, one per state frame):\n", run.size());
  std::printf("  min %.0f  mean %.2f  max %.0f   trend %+.3f entries/s", mn, mean, mx, slope);
  if (slope > 0.05) {
    std::printf("  -> would reach 255 in about %.0f s (latency then ~%d ms)", (255 - mean) / slope, 255 * a.cmd_t_ms);
  }
  std::printf("\n  per 10 s:");
  for (double w0 = 0; w0 < produced_s; w0 += 10) {
    double wmin = 1e9, wsum = 0, wmax = 0;
    int n = 0;
    for (const auto & s : run) {
      if (s.t >= w0 && s.t < w0 + 10) {wmin = std::min<double>(wmin, s.queue); wmax = std::max<double>(wmax, s.queue); wsum += s.queue; ++n;}
    }
    if (n > 0) {std::printf(" [%.0f/%.1f/%.0f]", wmin, wsum / n, wmax);}
  }
  std::printf(" (min/mean/max)");
  std::printf("\n  histogram:");
  for (const auto & kv : hist) {std::printf(" %d:%d", kv.first, kv.second);}
  std::printf("\n  robot_state went to 'stopped' %d time(s) while streaming (coarse underrun hint)\n", stops);
  std::printf("\nsender: sent=%llu rejected=%llu overflows=%llu refused-by-enqueue=%ld max_host_backlog=%llu\n",
    static_cast<unsigned long long>(st.sent), static_cast<unsigned long long>(st.rejected),
    static_cast<unsigned long long>(st.overflows), refused, static_cast<unsigned long long>(st.max_backlog));
  std::printf("drift trim at the end of the stream: %+.0f ppm (positive: the controller consumes slower than the host)\n",
    trim_ppm);
  std::printf("ServoJ call: mean %.0f us, max %llu us   produced %ld commands in %.2f s (%.2f ms mean)\n",
    st.mean_call_us, static_cast<unsigned long long>(st.max_call_us), ticks + 1, produced_s,
    produced_s * 1000.0 / (ticks + 1));

  // ---- lag & tracking error via cross-correlation ----
  if (!cmd_points.empty() && !samples.empty()) {
    const double t_start = cmd_points.front().t;
    const double t_end = cmd_points.back().t;
    const auto interp = [](const auto & vec, double t) {
      if (t <= vec.front().t) return vec.front().pos;
      if (t >= vec.back().t) return vec.back().pos;
      size_t l = 0, r = vec.size() - 1;
      while (l + 1 < r) {
        size_t mid = (l + r) / 2;
        if (vec[mid].t <= t) l = mid; else r = mid;
      }
      double frac = (t - vec[l].t) / (vec[r].t - vec[l].t);
      return vec[l].pos + frac * (vec[r].pos - vec[l].pos);
    };

    double max_err = 0;
    for (double t = t_start; t <= t_end; t += 0.002) {
      double c = interp(cmd_points, t);
      double f = interp(samples, t);
      max_err = std::max(max_err, std::abs(c - f));
    }

    int best_lag_ms = 0;
    double best_mse = 1e9;
    for (int lag = 0; lag <= 300; ++lag) {
      double dt = lag * 0.001;
      double mse = 0;
      int count = 0;
      for (double t = t_start; t <= t_end - dt; t += 0.002) {
        double c = interp(cmd_points, t);
        double f = interp(samples, t + dt);
        double diff = c - f;
        mse += diff * diff;
        ++count;
      }
      if (count > 0) {
        mse /= count;
        if (mse < best_mse) {
          best_mse = mse;
          best_lag_ms = lag;
        }
      }
    }
    std::printf("\ntracking & latency (J%d swing 0 to %.2f deg and back):\n", a.joint, a.amplitude_deg);
    std::printf("  max error between cmd and fb: %.3f deg (%.4f rad)\n", max_err, max_err * M_PI / 180.0);
    std::printf("  cross-correlation lag: %d ms (residual rms %.4f deg)\n", best_lag_ms, std::sqrt(best_mse));
  }

  std::printf("end: joint %d is %+.4f deg from the start, main_code=%d, ServoMoveEnd code=%d, comm error=%d\n",
    a.joint, final_err, snap.state.main_code, ended.code, static_cast<int>(sender.commError()));
  return sender.commError() == ApiError::None && snap.state.main_code == 0 ? 0 : 1;
}
