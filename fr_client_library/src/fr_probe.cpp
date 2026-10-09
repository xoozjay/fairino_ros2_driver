// Read-only probe: controller version, state-stream period and the live state.
// Usage: fr_probe <ip> [--watch]      (--watch: print once a second until Ctrl-C)
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>

#include "fr_client_library/robot_api.hpp"
#include "fr_client_library/state_receiver.hpp"

using namespace fr_client_library;

namespace
{
volatile std::sig_atomic_t g_stop = 0;

void printState(const StateSnapshot & s)
{
  const RobotState384 & st = s.state;
  const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - s.received_at).count();
  std::printf(
    "frame_cnt=%u frames=%llu age=%lldms robot_state=%u mode=%u main_code=%d sub_code=%d\n"
    "  estop=%u collision=%u si0=%u si1=%u enable=%d motion_done=%d\n"
    "  joints[deg]= %.3f %.3f %.3f %.3f %.3f %.3f\n"
    "  gripper: active=0x%04x position=%u%% fault=%u motion_done=%u\n",
    st.frame_cnt, static_cast<unsigned long long>(s.frame_count), static_cast<long long>(age_ms),
    st.robot_state, st.robot_mode, st.main_code, st.sub_code, st.EmergencyStop, st.collisionState,
    st.safety_stop0_state, st.safety_stop1_state, st.rbtEnableState, st.motion_done,
    st.jt_cur_pos[0], st.jt_cur_pos[1], st.jt_cur_pos[2], st.jt_cur_pos[3], st.jt_cur_pos[4],
    st.jt_cur_pos[5], st.gripper_active, st.gripper_position, st.gripper_fault,
    st.gripper_motiondone);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <ip> [--watch]\n", argv[0]);
    return 2;
  }
  const std::string ip = argv[1];
  const bool watch = argc > 2 && std::strcmp(argv[2], "--watch") == 0;
  std::signal(SIGINT, [](int) {g_stop = 1;});

  RobotApi api(ip);
  if (!api.connect()) {
    std::fprintf(stderr, "cannot connect to %s:20003\n", ip.c_str());
    return 1;
  }
  const auto info = api.getSoftwareVersion();
  SoftwareVersion version = kTargetVersion;
  if (info.ok()) {
    const auto & v = info.value;
    const char * verdict = !v.version ? "UNPARSEABLE version" :
      isVerifiedVersion(*v.version) ? "verified" : "NOT verified";
    std::printf("model=%s software=%s (%s) controller=%s\n", v.robot_model.c_str(),
      v.software_version.c_str(), verdict, v.controller_version.c_str());
    if (v.version && stateFrameSize(*v.version) != 0) {
      version = *v.version;
    } else {
      std::fprintf(stderr, "no known state frame size for this version, assuming %s\n",
        toString(kTargetVersion).c_str());
    }
  } else {
    std::fprintf(stderr, "GetSoftwareVersion failed: error=%d code=%d %s\n",
      static_cast<int>(info.status.error), info.status.code, info.status.message.c_str());
  }
  const auto period = api.getRealtimeStateSamplePeriod();
  if (period.ok()) {
    std::printf("state period=%d ms\n", period.value);
  }

  StateReceiver receiver(ip);
  if (!receiver.start(version) || !receiver.waitForFrame(2000)) {
    std::fprintf(stderr, "no state frame from %s:20004\n", ip.c_str());
    const auto st = receiver.stats();
    if (st.wrong_size_frames > 0) {
      const char * sw = describeFrameSize(st.last_wrong_size);
      std::fprintf(stderr, "frames of %zu bytes received (software %s?), expected %zu for %s\n",
        st.last_wrong_size, sw ? sw : "unknown", stateFrameSize(version),
        toString(version).c_str());
    }
    return 1;
  }

  StateSnapshot snap;
  do {
    if (receiver.latest(snap)) {
      printState(snap);
    }
    if (!receiver.running()) {
      std::fprintf(stderr, "state connection lost\n");
      return 1;
    }
    if (watch) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } while (watch && !g_stop);
  return 0;
}
