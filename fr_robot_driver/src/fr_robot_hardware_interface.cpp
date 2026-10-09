#include "fr_robot_driver/fr_robot_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <optional>

#include "fr_robot_driver/reset_policy.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

using fr_client_library::ApiError;
using fr_client_library::FaultClass;
using fr_client_library::InstantCommand;
using fr_client_library::InstantStatus;
using fr_client_library::RobotState384;
using fr_client_library::StateSnapshot;
using fr_robot_driver::FlangeMismatch;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr_robot_driver
{
namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
// The arm has arrived when every joint reads back within this of the point sent (sdk_quirks S18:
// standing, the readback stays within 0.002 deg)
constexpr double kAtRestDeg = 0.01;

// State interfaces of the <gpio name="<tf_prefix>fr_status"> component (docs/fault_handling.md 7).
constexpr const char * kStatusInterfaces[] = {
  "fault_class", "estop", "safety_stop0", "safety_stop1", "collision", "main_code", "sub_code",
  "enabled", "robot_mode", "robot_state", "servo_queue_len"};

}  // namespace

FrRobotHardwareInterface::~FrRobotHardwareInterface()
{
  // the node's callbacks first: they use the connections
  if (node_) {
    if (auto exec = executor_.lock()) {
      exec->remove_node(node_->get_node_base_interface());
    }
  }
  joinCleanup();
  if (receiver_) {receiver_->stop();}   // null only if on_init failed early
  if (sender_) {sender_->stop();}
}

std::string FrRobotHardwareInterface::param(const std::string & key, const std::string & fallback) const
{
  const auto & p = get_hardware_info().hardware_parameters;
  const auto it = p.find(key);
  return it == p.end() ? fallback : it->second;
}

CallbackReturn FrRobotHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();
  const auto log = get_logger();

  // ---- joints: six, position command, position + velocity state, a URDF velocity limit ----
  if (info.joints.size() != 6) {
    RCLCPP_ERROR(log, "expected 6 joints, the URDF declares %zu", info.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : info.joints) {
    const bool pos_cmd = joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION;
    const auto has_state = [&](const char * n) {
        return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                 [&](const auto & i) {return i.name == n;});
      };
    if (!pos_cmd || !has_state(hardware_interface::HW_IF_POSITION) ||
      !has_state(hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(log, "joint '%s': need command 'position' and states 'position', 'velocity'",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    const auto lim = info.limits.find(joint.name);
    if (lim == info.limits.end() || !lim->second.has_velocity_limits) {
      RCLCPP_ERROR(log, "joint '%s' has no velocity limit in the URDF (needed for the step check)",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  // ---- gpio fr_status ----
  status_prefix_ = param("tf_prefix", "") + "fr_status";
  const auto gpio = std::find_if(info.gpios.begin(), info.gpios.end(),
      [&](const auto & g) {return g.name == status_prefix_;});
  if (gpio == info.gpios.end()) {
    RCLCPP_ERROR(log, "the URDF has no <gpio name=\"%s\">", status_prefix_.c_str());
    return CallbackReturn::ERROR;
  }
  for (const char * want : kStatusInterfaces) {
    const bool found = std::any_of(gpio->state_interfaces.begin(), gpio->state_interfaces.end(),
        [&](const auto & i) {return i.name == want;});
    if (!found) {
      RCLCPP_ERROR(log, "<gpio name=\"%s\"> lacks the state interface '%s'", status_prefix_.c_str(), want);
      return CallbackReturn::ERROR;
    }
  }
  status_prefix_ += "/";

  // ---- parameters ----
  bool ok = true;
  const auto integer = [&](const char * key, int fallback, int lo, int hi) {
      try {
        const int v = std::stoi(param(key, std::to_string(fallback)));
        if (v >= lo && v <= hi) {return v;}
      } catch (const std::exception &) {
      }
      RCLCPP_ERROR(log, "parameter '%s' must be an integer in [%d, %d]", key, lo, hi);
      ok = false;
      return fallback;
    };
  const auto number = [&](const char * key, double fallback, double lo, double hi) {
      try {
        const double v = std::stod(param(key, std::to_string(fallback)));
        if (v >= lo && v <= hi) {return v;}
      } catch (const std::exception &) {
      }
      RCLCPP_ERROR(log, "parameter '%s' must be a number in [%g, %g]", key, lo, hi);
      ok = false;
      return fallback;
    };
  ip_ = param("ip", "192.168.58.2");
  xmlrpc_port_ = static_cast<uint16_t>(integer("xmlrpc_port", 20003, 1, 65535));
  state_port_ = static_cast<uint16_t>(integer("state_port", 20004, 1, 65535));
  instant_port_ = static_cast<uint16_t>(integer("instant_port", 8080, 1, 65535));
  cmd_t_ms_ = integer("cmd_t_ms", 8, 1, 1000);
  prefill_ = integer("prefill", 3, 0, 64);
  state_period_ms_ = integer("state_period_ms", 8, 8, 1000);  // simulator: below 8 is refused
  comm_timeout_periods_ = integer("comm_timeout_periods", 3, 1, 1000);
  const int update_rate = integer("update_rate_hz", 1000 / std::max(1, cmd_t_ms_), 1, 1000);
  allow_unverified_version_ = hardware_interface::parse_bool(param("allow_unverified_version", "false"));
  model_ = param("model", "");
  allow_unknown_model_ = hardware_interface::parse_bool(param("allow_unknown_model", "false"));
  fk_check_ = hardware_interface::parse_bool(param("fk_check", "true"));
  fk_position_tolerance_mm_ = number("fk_position_tolerance_mm", 10.0, 0.01, 1000.0);
  fk_angle_tolerance_deg_ = number("fk_angle_tolerance_deg", 1.0, 0.001, 180.0);
  if (model_.empty()) {
    RCLCPP_ERROR(log, "parameter 'model' (the fr_description model, e.g. fairino5_v6) is required");
    ok = false;
  }
  if (fk_check_) {
    const double flange = number("flange_offset", -1.0, 0.0, 1.0);
    const auto k = parseKinematics(param("kinematics", ""), flange);
    if (!k) {
      RCLCPP_ERROR(log, "parameters 'kinematics' (six \"x y z r p y\" joint origins) and "
        "'flange_offset' are required for the flange check (fk_check)");
      ok = false;
    } else {
      kinematics_ = *k;
    }
  }
  if (!ok) {
    return CallbackReturn::ERROR;
  }
  // One ServoJ segment per ratio_ controller cycles; must divide evenly (docs/sdk_quirks.md S1).
  if ((update_rate * cmd_t_ms_) % 1000 != 0 || update_rate * cmd_t_ms_ < 1000) {
    RCLCPP_ERROR(log, "update_rate_hz=%d and cmd_t_ms=%d: cmdT must be a whole number of controller "
      "cycles (update_rate_hz * cmd_t_ms / 1000 must be an integer >= 1)", update_rate, cmd_t_ms_);
    return CallbackReturn::ERROR;
  }
  ratio_ = update_rate * cmd_t_ms_ / 1000;
  for (const auto & name : joint_names_) {
    const double vmax_deg_s = info.limits.at(name).max_velocity * kRadToDeg;
    max_step_deg_.push_back(vmax_deg_s * cmd_t_ms_ / 1000.0);
  }

  // ---- connections (see the header): created here, connected in on_configure ----
  api_ = std::make_unique<fr_client_library::RobotApi>(
    ip_, xmlrpc_port_, fr_client_library::HttpOptions{1000, 1000});
  receiver_ = std::make_unique<fr_client_library::StateReceiver>(ip_, state_port_);
  instant_ = std::make_unique<fr_client_library::InstantCommandChannel>(ip_, instant_port_);
  fr_client_library::ServoSenderOptions sender_options;
  sender_options.cmd_t_ms = cmd_t_ms_;
  sender_options.prefill = prefill_;
  // The sender thread may run at SCHED_FIFO: its callbacks never wait for the receiver and keep
  // their previous frame when it is busy. A stale frame is no information.
  // Drift compensation (sdk_quirks S17): the firmware queue depth.
  sender_options.queue_depth = [this, snap = StateSnapshot{}]() mutable -> std::optional<int> {
      receiver_->tryLatest(snap);
      if (!snap.fresh(comm_timeout_)) {
        return std::nullopt;
      }
      return fr_client_library::servoQueueLength(snap.state);
    };
  // Idle pause (sdk_quirks S18): nothing is sent while the arm stands where it was sent to.
  sender_options.at_rest = [this, snap = StateSnapshot{}](const JointArray & t) mutable {
      receiver_->tryLatest(snap);
      if (!snap.fresh(comm_timeout_)) {
        return false;
      }
      for (int i = 0; i < 6; ++i) {
        if (std::abs(snap.state.jt_cur_pos[i] - t[i]) > kAtRestDeg) {
          return false;
        }
      }
      return true;
    };
  sender_ = std::make_unique<fr_client_library::ServoSender>(ip_, xmlrpc_port_, sender_options);

  // ---- reset_error service on the controller manager's executor (optional: no executor in tests)
  executor_ = params.executor;
  if (auto exec = executor_.lock()) {
    node_ = std::make_shared<rclcpp::Node>(info.name + "_hw", param("namespace", ""));
    reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reset_error",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onResetRequest(req, res);});
    exec->add_node(node_->get_node_base_interface());
  } else {
    RCLCPP_WARN(log, "no executor: the reset_error service is not available");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  const auto log = get_logger();
  joinCleanup();
  closeConnections();
  fault_class_ = FaultClass::None;
  step_fault_ = false;
  logged_class_ = FaultClass::None;
  frame_ = StateSnapshot{};

  try {
    {
      std::lock_guard<std::mutex> lock(api_mutex_);
      if (!api_->connect()) {
        RCLCPP_ERROR(log, "cannot connect to %s:%d (XML-RPC)", ip_.c_str(), xmlrpc_port_);
        return CallbackReturn::FAILURE;
      }
      const auto info = api_->getSoftwareVersion();
      if (!info.ok() || !info.value.version) {
        RCLCPP_ERROR(log, "GetSoftwareVersion failed or returned an unparseable version ('%s')",
          info.value.software_version.c_str());
        return CallbackReturn::FAILURE;
      }
      const fr_client_library::SoftwareVersion version = *info.value.version;
      version_ = version;
      RCLCPP_INFO(log, "controller: model %s, software %s, controller version %s",
        info.value.robot_model.c_str(), info.value.software_version.c_str(),
        info.value.controller_version.c_str());
      if (!fr_client_library::isVerifiedVersion(version)) {
        if (!allow_unverified_version_) {
          RCLCPP_ERROR(log, "software version %s is not verified with this driver "
            "(set allow_unverified_version to run anyway)", info.value.software_version.c_str());
          return CallbackReturn::FAILURE;
        }
        RCLCPP_WARN(log, "software version %s is NOT verified, running anyway",
          info.value.software_version.c_str());
      }
      if (fr_client_library::stateFrameSize(version) == 0) {
        RCLCPP_ERROR(log, "no known state frame size for software version %s",
          info.value.software_version.c_str());
        return CallbackReturn::FAILURE;
      }

      // Which robot is it? The URDF (limits, kinematics) was built for model_.
      if (const char * described = fr_client_library::descriptionModelFor(info.value.robot_model)) {
        if (model_ != described) {
          RCLCPP_ERROR(log, "the controller is a %s (%s) but this URDF was built for %s",
            info.value.robot_model.c_str(), described, model_.c_str());
          return CallbackReturn::FAILURE;
        }
      } else if (!allow_unknown_model_) {
        RCLCPP_ERROR(log, "controller model '%s' is not one this driver has kinematics for "
          "(allow_unknown_model:=true to run anyway; the flange check still applies)",
          info.value.robot_model.c_str());
        return CallbackReturn::FAILURE;
      } else {
        RCLCPP_WARN(log, "controller model '%s' is unknown, assuming %s",
          info.value.robot_model.c_str(), model_.c_str());
      }

      // state period: set and read back (the controller refuses < 8 ms)
      auto period = api_->getRealtimeStateSamplePeriod();
      if (!period.ok() || period.value != state_period_ms_) {
        if (const auto st = api_->setRealtimeStateSamplePeriod(state_period_ms_); !st.ok()) {
          RCLCPP_ERROR(log, "SetRobotRealtimeStateSamplePeriod(%d) failed (error %d, code %d)",
            state_period_ms_, static_cast<int>(st.error), st.code);
          return CallbackReturn::FAILURE;
        }
        period = api_->getRealtimeStateSamplePeriod();
      }
      if (!period.ok() || period.value != state_period_ms_) {
        RCLCPP_ERROR(log, "state period read back as %d ms, wanted %d", period.value, state_period_ms_);
        return CallbackReturn::FAILURE;
      }
    }

    if (!receiver_->start(version_) || !receiver_->waitForFrame(2000)) {
      RCLCPP_ERROR(log, "no state frames from %s:%d", ip_.c_str(), state_port_);
      closeConnections();
      return CallbackReturn::FAILURE;
    }
    comm_timeout_ = std::chrono::milliseconds(state_period_ms_ * comm_timeout_periods_);
    {
      StateSnapshot first;
      std::string why;
      if (!receiver_->latest(first) || !flangeConsistent(first.state, why)) {
        RCLCPP_ERROR(log, "flange check failed: %s", why.c_str());
        closeConnections();
        return CallbackReturn::FAILURE;
      }
    }

    // 8080 has room for one client only: this driver must be its sole user (sdk_quirks L8)
    if (!instant_->connect()) {
      RCLCPP_ERROR(log, "cannot connect to %s:%d (instant commands; only one client is served)",
        ip_.c_str(), instant_port_);
      closeConnections();
      return CallbackReturn::FAILURE;
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(log, "configure failed: %s", e.what());
    closeConnections();
    return CallbackReturn::FAILURE;
  }
  configured_ = true;
  RCLCPP_INFO(log, "configured: %s, ServoJ every %d ms (every %d controller cycle(s)), prefill %d",
    ip_.c_str(), cmd_t_ms_, ratio_, prefill_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  const auto log = get_logger();
  joinCleanup();  // a previous deactivation may still be closing the servo session

  // Preconditions (docs/fault_handling.md 4): never reset, never enable. The state comes from the
  // receiver, which any thread may read: on_activate may run before the first read() (startup).
  StateSnapshot snap;
  receiver_->latest(snap);
  if (!snap.fresh(comm_timeout_)) {
    RCLCPP_ERROR(log, "cannot activate: no fresh state frame");
    return CallbackReturn::FAILURE;
  }
  if (const FaultClass fc = fr_client_library::classifyState(snap.state); fc != FaultClass::None) {
    RCLCPP_ERROR(log, "cannot activate: %s (%s)", fr_client_library::toString(fc),
      fr_client_library::describeFault(snap.state).c_str());
    return CallbackReturn::FAILURE;
  }

  if (std::string why; !flangeConsistent(snap.state, why)) {
    RCLCPP_ERROR(log, "cannot activate: flange check failed: %s", why.c_str());
    return CallbackReturn::FAILURE;
  }

  JointArray actual;
  for (int i = 0; i < 6; ++i) {
    actual[i] = snap.state.jt_cur_pos[i];
  }
  // Commands start at the actual position, never at zero (docs R3).
  for (int i = 0; i < 6; ++i) {
    set_command(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, actual[i] * kDegToRad);
  }
  const auto st = sender_->start(actual);
  if (!st.ok()) {
    RCLCPP_ERROR(log, "cannot start the servo stream (error %d, code %d %s)",
      static_cast<int>(st.error), st.code, st.message.c_str());
    return CallbackReturn::FAILURE;
  }
  last_cmd_deg_ = actual;
  write_count_ = 0;
  step_fault_ = false;
  active_ = true;
  RCLCPP_INFO(log, "activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // May run on the realtime thread (write() returned DEACTIVATE): no blocking here.
  active_ = false;
  sender_->halt();
  // With a fault the firmware stops by itself; otherwise flush its queue with STOP first (S15, S16).
  startCleanup(fault_class_.load() == FaultClass::None && !step_fault_.load(), false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  joinCleanup();
  closeConnections();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  joinCleanup();
  closeConnections();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
  // Runs inside read()/write() (realtime thread). Stop, close, do not reset, do not reconnect.
  active_ = false;
  configured_ = false;
  sender_->halt();
  startCleanup(false, true);
  return CallbackReturn::SUCCESS;  // -> UNCONFIGURED
}

bool FrRobotHardwareInterface::flangeConsistent(const RobotState384 & s, std::string & why) const
{
  if (!fk_check_) {
    return true;
  }
  std::array<double, 6> q;
  double flange[6];  // copies: the frame is packed
  for (int i = 0; i < 6; ++i) {
    q[i] = s.jt_cur_pos[i];
    flange[i] = s.flange_cur_pos[i];
  }
  const FlangeMismatch m = compareFlange(kinematics_, q, flange);
  if (m.position_mm <= fk_position_tolerance_mm_ && m.angle_deg <= fk_angle_tolerance_deg_) {
    RCLCPP_INFO(get_logger(), "flange check: %.3f mm / %.4f deg from FK (limits %.3f mm / %.3f deg)",
      m.position_mm, m.angle_deg, fk_position_tolerance_mm_, fk_angle_tolerance_deg_);
    return true;
  }
  why = "the controller's flange pose is " + std::to_string(m.position_mm) + " mm / " +
    std::to_string(m.angle_deg) + " deg away from the forward kinematics of " + model_ +
    " (limits " + std::to_string(fk_position_tolerance_mm_) + " mm / " +
    std::to_string(fk_angle_tolerance_deg_) + " deg): wrong robot model, joint zero offsets or "
    "installation? (fk_check:=false to disable)";
  return false;
}

void FrRobotHardwareInterface::startCleanup(bool send_stop, bool close_all)
{
  if (cleanup_thread_.joinable()) {
    cleanup_thread_.join();
  }
  cleanup_thread_ = std::thread([this, send_stop, close_all] {
      try {
        if (send_stop && instant_->connected()) {
          const auto r = instant_->send(InstantCommand::Stop);
          if (r.status != InstantStatus::Ok) {
            RCLCPP_ERROR(get_logger(), "STOP before deactivation failed (status %d)",
              static_cast<int>(r.status));
          }
        }
        sender_->stop();  // ServoMoveEnd: not a sign that the robot stopped (S6)
        if (close_all) {
          closeConnections();
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "cleanup thread ended early: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "cleanup thread ended early: unknown exception");
      }
    });
}

void FrRobotHardwareInterface::joinCleanup()
{
  if (cleanup_thread_.joinable()) {
    cleanup_thread_.join();
  }
}

void FrRobotHardwareInterface::closeConnections()
{
  configured_ = false;
  sender_->stop();
  receiver_->stop();
  instant_->disconnect();
  std::lock_guard<std::mutex> lock(api_mutex_);
  api_->close();
}

void FrRobotHardwareInterface::logFault(FaultClass fc, const std::string & detail)
{
  if (fc == logged_class_) {
    return;
  }
  const auto log = get_logger();
  if (fc == FaultClass::None) {
    RCLCPP_INFO(log, "fault cleared");
  } else {
    RCLCPP_ERROR(log, "%s: %s", fr_client_library::toString(fc), detail.c_str());
  }
  logged_class_ = fc;
}

void FrRobotHardwareInterface::publishJoints(const RobotState384 & s)
{
  for (int i = 0; i < 6; ++i) {
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, s.jt_cur_pos[i] * kDegToRad);
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, s.actual_qd[i] * kDegToRad);
  }
}

void FrRobotHardwareInterface::publishStatus(const RobotState384 & s, FaultClass fc)
{
  const auto put = [this](const char * name, double v) {set_state(status_prefix_ + name, v);};
  put("fault_class", static_cast<double>(fc));
  put("estop", s.EmergencyStop);
  put("safety_stop0", s.safety_stop0_state);
  put("safety_stop1", s.safety_stop1_state);
  put("collision", s.collisionState);
  put("main_code", s.main_code);   // error codes are exported as they are, never translated
  put("sub_code", s.sub_code);
  put("enabled", s.rbtEnableState == 1 ? 1.0 : 0.0);
  put("robot_mode", s.robot_mode);
  put("robot_state", s.robot_state);
  put("servo_queue_len", fr_client_library::servoQueueLength(s));
}

return_type FrRobotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  receiver_->tryLatest(frame_);   // the receiver is busy this instant: keep the previous frame
  const StateSnapshot & snap = frame_;
  const bool stale = !snap.fresh(comm_timeout_);
  const bool link_lost = !receiver_->running();
  const bool sender_dead = active_.load() &&
    (!sender_->running() || sender_->commError() != ApiError::None);

  // COMM: the link cannot be trusted, so nothing in the frames can be (R3, R5)
  if (stale || link_lost || sender_dead) {
    fault_class_ = FaultClass::Comm;
    set_state(status_prefix_ + "fault_class", static_cast<double>(FaultClass::Comm));
    logFault(FaultClass::Comm, stale ? "no state frame in time" :
      link_lost ? "state connection lost" : "servo stream connection failed");
    return return_type::ERROR;  // -> on_error -> UNCONFIGURED
  }

  const RobotState384 & s = snap.state;

  FaultClass fc = fr_client_library::classifyState(s);
  std::string detail = fr_client_library::describeFault(s);
  if (step_fault_.load() && fc == FaultClass::None) {
    fc = FaultClass::RobotFault;  // our own protective stop stays until the next activation
    detail = "a command was refused by the step check";
  }
  fault_class_ = fc;
  publishJoints(s);
  publishStatus(s, fc);
  if (active_.load() && fc != FaultClass::None) {
    sender_->halt();  // stop at once; write() answers DEACTIVATE
  }
  logFault(fc, detail);
  return return_type::OK;
}

return_type FrRobotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {
    return return_type::OK;
  }
  if (fault_class_.load() != FaultClass::None) {
    sender_->halt();
    return return_type::DEACTIVATE;  // -> INACTIVE; the operator resets and reactivates (F3)
  }
  if (write_count_++ % static_cast<unsigned long>(ratio_) != 0) {
    return return_type::OK;  // controller runs faster than cmdT: only every ratio_-th cycle is a segment
  }

  JointArray target;
  for (int i = 0; i < 6; ++i) {
    target[i] = get_command<double>(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION) * kRadToDeg;
  }
  // Step check (docs/fault_handling.md 6): the firmware only limits speed/acceleration/range itself
  // (sdk_quirks S14) and a violation puts the robot in a fault (S11).
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(target[i]) || std::abs(target[i] - last_cmd_deg_[i]) > max_step_deg_[i]) {
      RCLCPP_ERROR(get_logger(), "command refused: joint %s asks for %.3f deg after %.3f deg in one "
        "%d ms step (limit %.3f deg)", joint_names_[i].c_str(), target[i], last_cmd_deg_[i],
        cmd_t_ms_, max_step_deg_[i]);
      step_fault_ = true;
      fault_class_ = FaultClass::RobotFault;
      sender_->halt();
      return return_type::DEACTIVATE;
    }
  }
  if (!sender_->enqueue(target)) {
    // full queue or dead sender: the stream to the controller is broken
    fault_class_ = FaultClass::Comm;
    return return_type::ERROR;
  }
  last_cmd_deg_ = target;
  return return_type::OK;
}

void FrRobotHardwareInterface::onResetRequest(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(api_mutex_);  // its own XML-RPC connection, not the servo one
  const ResetDecision decision = decideReset(configured_.load(), fault_class_.load());
  if (decision != ResetDecision::Allowed) {
    response->success = false;
    response->message = describe(decision);
    RCLCPP_WARN(get_logger(), "reset_error refused: %s", response->message.c_str());
    return;
  }
  const auto st = api_->resetAllError();
  response->success = st.ok();
  response->message = st.ok() ? "ResetAllError sent; reactivate the hardware component to continue" :
    "ResetAllError failed (error " + std::to_string(static_cast<int>(st.error)) + ", code " +
    std::to_string(st.code) + ")";
  RCLCPP_INFO(get_logger(), "reset_error: %s", response->message.c_str());
}

}  // namespace fr_robot_driver

PLUGINLIB_EXPORT_CLASS(fr_robot_driver::FrRobotHardwareInterface, hardware_interface::SystemInterface)
