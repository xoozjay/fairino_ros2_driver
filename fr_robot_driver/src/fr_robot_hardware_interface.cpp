#include "fr_robot_driver/fr_robot_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <initializer_list>
#include <iterator>
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

// ---- gripper: DH-Robotics PGI-140-80 (gripper_dahuan_pgi140); the constants are in GripperGoals ----
constexpr double kStillDegPerS = 0.5;  // the arm counts as standing still below this joint speed
// The arm has arrived when every joint reads back within this of the point sent (sdk_quirks S18:
// standing, the readback stays within 0.002 deg)
constexpr double kAtRestDeg = 0.01;
constexpr auto kGripperActivationTimeout = std::chrono::seconds(3);

}  // namespace

FrRobotHardwareInterface::~FrRobotHardwareInterface()
{
  // the node's callbacks first: they use the connections
  if (status_timer_) {status_timer_->cancel();}
  if (node_) {
    if (auto exec = executor_.lock()) {
      exec->remove_node(node_->get_node_base_interface());
    }
  }
  joinCleanup();
  if (receiver_) {receiver_->stop();}   // null only if on_init failed early
  if (sender_) {sender_->stop();}
  if (gripper_sender_) {gripper_sender_->stop();}
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

  const auto has = [](const std::vector<hardware_interface::InterfaceInfo> & list, const std::string & n) {
      return std::any_of(list.begin(), list.end(), [&](const auto & i) {return i.name == n;});
    };

  const auto interfaces = [&](const auto & components, const std::string & name,
      std::initializer_list<const char *> commands, std::initializer_list<const char *> states) {
      const auto c = std::find_if(components.begin(), components.end(),
          [&](const auto & item) {return item.name == name;});
      return c != components.end() &&
        std::all_of(commands.begin(), commands.end(),
        [&](const auto * n) {return has(c->command_interfaces, n);}) &&
        std::all_of(states.begin(), states.end(),
        [&](const auto * n) {return has(c->state_interfaces, n);});
    };

  // ---- optional gripper joint: opening [m] commanded with max_effort [N] ----
  gripper_joint_ = param("gripper_joint", "");
  if (hasGripper()) {
    if (!interfaces(info.joints, gripper_joint_, {"position", "max_effort"}, {"position", "velocity"}))
    {
      RCLCPP_ERROR(log, "gripper joint '%s': need commands 'position', 'max_effort' "
        "and states 'position', 'velocity'", gripper_joint_.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // ---- arm joints: six, position command, position + velocity state, a URDF velocity limit ----
  std::vector<hardware_interface::ComponentInfo> arm_joints;
  std::copy_if(info.joints.begin(), info.joints.end(), std::back_inserter(arm_joints),
    [&](const auto & j) {return j.name != gripper_joint_;});
  if (arm_joints.size() != 6) {
    RCLCPP_ERROR(log, "expected 6 arm joints, the URDF declares %zu", arm_joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : arm_joints) {
    const bool pos_cmd = joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION;
    if (!pos_cmd || !has(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
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

  // ---- gpio fr_status: the fault class only; everything else is in `~/status` (docs/fault_handling.md 7)
  const std::string gpio_name = param("tf_prefix", "") + "fr_status";
  if (!interfaces(info.gpios, gpio_name, {}, {"fault_class"})) {
    RCLCPP_ERROR(log, "the URDF needs <gpio name=\"%s\"> with the state interface 'fault_class'",
      gpio_name.c_str());
    return CallbackReturn::ERROR;
  }
  fault_class_interface_ = gpio_name + "/fault_class";

  // ---- gpio speed_scaling: 0 pauses the joint_trajectory_controller while the gripper is busy ----
  const std::string scaling_name = param("tf_prefix", "") + "speed_scaling";
  if (!interfaces(info.gpios, scaling_name, {}, {"speed_scaling_factor"})) {
    RCLCPP_ERROR(log, "the URDF needs <gpio name=\"%s\"> with the state interface 'speed_scaling_factor'",
      scaling_name.c_str());
    return CallbackReturn::ERROR;
  }
  speed_scaling_interface_ = scaling_name + "/speed_scaling_factor";

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
  rt_priority_ = integer("rt_priority", 0, 0, 99);
  state_period_ms_ = integer("state_period_ms", 8, 8, 1000);  // simulator: below 8 is refused
  comm_timeout_periods_ = integer("comm_timeout_periods", 3, 1, 1000);
  comm_timeout_ = std::chrono::milliseconds(state_period_ms_ * comm_timeout_periods_);
  const int update_rate = integer("update_rate_hz", 1000 / std::max(1, cmd_t_ms_), 1, 1000);
  allow_unverified_version_ = hardware_interface::parse_bool(param("allow_unverified_version", "false"));
  model_ = param("model", "");
  allow_unknown_model_ = hardware_interface::parse_bool(param("allow_unknown_model", "false"));
  fk_check_ = hardware_interface::parse_bool(param("fk_check", "true"));
  fk_position_tolerance_mm_ = number("fk_position_tolerance_mm", 10.0, 0.01, 1000.0);
  fk_angle_tolerance_deg_ = number("fk_angle_tolerance_deg", 1.0, 0.001, 180.0);
  gripper_index_ = integer("gripper_index", 1, 1, 16);   // active bit index - 1 (sdk_quirks G5)
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
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const double vmax_deg_s = info.limits.at(joint_names_[i]).max_velocity * kRadToDeg;
    max_step_deg_[i] = vmax_deg_s * cmd_t_ms_ / 1000.0;
  }

  // ---- connections (see the header): created here, connected in on_configure ----
  api_ = std::make_unique<fr_client_library::RobotApi>(
    ip_, xmlrpc_port_, fr_client_library::HttpOptions{1000, 1000});
  receiver_ = std::make_unique<fr_client_library::StateReceiver>(ip_, state_port_);
  instant_ = std::make_unique<fr_client_library::InstantCommandChannel>(ip_, instant_port_);
  fr_client_library::ServoSenderOptions sender_options;
  sender_options.cmd_t_ms = cmd_t_ms_;
  sender_options.prefill = prefill_;
  sender_options.rt_priority = rt_priority_;
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
  if (hasGripper()) {
    // Its own 20003 connection: MoveGripper never waits behind ServoJ or the services' calls.
    gripper_sender_ = std::make_unique<fr_client_library::GripperSender>(
      ip_, xmlrpc_port_, gripper_index_);
  }

  // ---- reset_error service on the controller manager's executor (optional: no executor in tests)
  executor_ = params.executor;
  if (auto exec = executor_.lock()) {
    node_ = std::make_shared<rclcpp::Node>(info.name + "_hw", param("namespace", ""));
    reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reset_error",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onResetRequest(req, res);});
    if (hasGripper()) {
      gripper_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      gripper_activate_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/gripper/activate",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onGripperActivate(req, res);},
        rclcpp::ServicesQoS(), gripper_group_);
      // relative name: /<namespace>/gripper_controller/gripper_cmd, where the mock's
      // parallel_gripper_action_controller has it
      gripper_action_ = rclcpp_action::create_server<GripperAction>(
        node_, "gripper_controller/gripper_cmd",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperAction::Goal> goal) {
          const auto & c = goal->command;
          if (c.position.size() != 1 || !std::isfinite(c.position[0])) {
            RCLCPP_WARN(get_logger(), "gripper goal rejected: needs exactly one finite position");
            return rclcpp_action::GoalResponse::REJECT;
          }
          if (!active_.load() || !gripper_usable_.load()) {
            RCLCPP_WARN(get_logger(), "gripper goal rejected: the hardware is not active, or the "
              "gripper is not active or reports a fault");
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GripperGoalHandle> &) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](const std::shared_ptr<GripperGoalHandle> handle) {onGripperGoalAccepted(handle);});
      gripper_goal_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20), [this] {onGripperGoalTimer();});
    }
    // `~/status`: lives as long as this node, whatever the hardware's lifecycle state is
    status_tracker_.identify(info.name, ip_);
    status_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "~/status", rclcpp::QoS(1).transient_local());
    publishDiagnostics();
    status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this] {publishDiagnostics();});
    exec->add_node(node_->get_node_base_interface());
  } else {
    RCLCPP_WARN(log, "no executor: the reset_error service and the status topic are not available");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  const auto log = get_logger();
  joinCleanup();   // also ends the status link monitor, which holds the state port
  closeConnections();
  fault_class_ = FaultClass::None;
  comm_reason_ = CommReason::None;
  protective_stop_ = ProtectiveStop::None;
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
    {
      StateSnapshot first;
      std::string why;
      if (!receiver_->latest(first) || !flangeConsistent(first.state, why)) {
        RCLCPP_ERROR(log, "flange check failed: %s", why.c_str());
        closeConnections();
        return CallbackReturn::FAILURE;
      }
      // What the controller assumes is mounted. Read only: the driver never writes it.
      const int tool = first.state.tool;
      std::lock_guard<std::mutex> lock(api_mutex_);
      const auto w = api_->getTargetPayload();
      const auto cog = api_->getTargetPayloadCog();
      const auto tcp = api_->getTcpOffset();
      if (w.ok() && cog.ok() && tcp.ok()) {
        RCLCPP_INFO(log, "controller payload %.3f kg, CoG [%.1f %.1f %.1f] mm; tool %d, TCP "
          "[%.1f %.1f %.1f] mm [%.2f %.2f %.2f] deg", w.value, cog.value[0], cog.value[1],
          cog.value[2], tool, tcp.value[0], tcp.value[1], tcp.value[2], tcp.value[3], tcp.value[4],
          tcp.value[5]);
      } else {
        RCLCPP_WARN(log, "could not read payload/tool from the controller (errors %d/%d/%d)",
          static_cast<int>(w.status.error), static_cast<int>(cog.status.error),
          static_cast<int>(tcp.status.error));
      }
    }

    // 8080 has room for one client only: this driver must be its sole user (sdk_quirks L8)
    if (!instant_->connect()) {
      RCLCPP_ERROR(log, "cannot connect to %s:%d (instant commands; only one client is served)",
        ip_.c_str(), instant_port_);
      closeConnections();
      return CallbackReturn::FAILURE;
    }

    if (hasGripper()) {
      if (!gripper_sender_->start()) {
        RCLCPP_ERROR(log, "cannot connect to %s:%d (XML-RPC, gripper)", ip_.c_str(), xmlrpc_port_);
        closeConnections();
        return CallbackReturn::FAILURE;
      }
      StateSnapshot snap;
      receiver_->latest(snap);
      RCLCPP_INFO(log, "gripper %d: %s, position %u %%, motion done %u, fault %u", gripper_index_,
        gripperActive(snap.state) ? "active" : "NOT active (activate with ~/gripper/activate)",
        snap.state.gripper_position, snap.state.gripper_motiondone, snap.state.gripper_fault);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(log, "configure failed: %s", e.what());
    closeConnections();
    return CallbackReturn::FAILURE;
  }
  configured_ = true;
  setHardwareState(HardwareState::Inactive);
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
  if (rt_priority_ > 0) {
    if (sender_->realtimeGranted()) {
      RCLCPP_INFO(log, "servo sender thread runs at SCHED_FIFO %d", rt_priority_);
    } else {
      RCLCPP_WARN(log, "servo sender: SCHED_FIFO %d refused (needs CAP_SYS_NICE or an rtprio "
        "limit); running with the normal scheduler", rt_priority_);
    }
  }
  if (hasGripper()) {
    ++gripper_epoch_;   // goals from before never run
    gripper_busy_ = GripperBusy::Idle;
    stream_idle_frames_ = 0;
    gripper_motion_.resetMotion();
  }
  gate_ = std::make_unique<ArmGate>(ratio_, actual, max_step_deg_);
  protective_stop_ = ProtectiveStop::None;
  active_ = true;
  setHardwareState(HardwareState::Active);
  RCLCPP_INFO(log, "activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // May run on the realtime thread (write() returned DEACTIVATE): no blocking here.
  active_ = false;
  setHardwareState(HardwareState::Inactive);
  sender_->halt();
  if (hasGripper()) {
    ++gripper_epoch_;
    gripper_sender_->discard();
  }
  // With a fault the firmware stops by itself; otherwise flush its queue with STOP first (S15, S16).
  startCleanup(fault_class_.load() == FaultClass::None &&
    protective_stop_.load() == ProtectiveStop::None, false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  joinCleanup();
  closeConnections();
  comm_reason_ = CommReason::None;   // the operator's own decision, not a fault
  setHardwareState(HardwareState::Unconfigured);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  joinCleanup();
  closeConnections();
  setHardwareState(HardwareState::Finalized);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrRobotHardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
  // Runs inside read()/write() (realtime thread). Stop, close, do not reset, do not reconnect the
  // robot. Only the read-only state stream is watched again (monitorLink) for the status topic.
  active_ = false;
  configured_ = false;
  setHardwareState(HardwareState::Unconfigured);
  sender_->halt();
  if (hasGripper()) {
    ++gripper_epoch_;
    gripper_sender_->discard();
  }
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
  joinCleanup();
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    cleanup_stop_ = false;
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
          monitorLink();  // returns when joinCleanup() asks
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "cleanup thread ended early: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "cleanup thread ended early: unknown exception");
      }
      monitoring_ = false;
    });
}

void FrRobotHardwareInterface::joinCleanup()
{
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    cleanup_stop_ = true;
  }
  stop_cv_.notify_all();
  if (cleanup_thread_.joinable()) {
    cleanup_thread_.join();
  }
}

void FrRobotHardwareInterface::monitorLink()
{
  monitoring_ = true;
  std::unique_lock<std::mutex> lock(stop_mutex_);
  while (!cleanup_stop_) {
    if (!receiver_->running()) {
      lock.unlock();
      receiver_->start(version_);  // fails quietly while the controller is away
      lock.lock();
    }
    stop_cv_.wait_for(lock, std::chrono::seconds(2), [this] {return cleanup_stop_;});
  }
  lock.unlock();
  receiver_->stop();  // frees the single-client state port for the next configure
}

void FrRobotHardwareInterface::publishDiagnostics()
{
  StatusInput in;
  in.hardware = hardware_state_.load();
  in.comm = comm_reason_.load();
  in.protective_stop = protective_stop_.load();
  in.stream_paused = active_.load() && sender_->paused();
  if (hasGripper()) {
    in.gripper.present = true;
    in.gripper.index = gripper_index_;
    in.gripper.source = gripper_source_.load();
    in.gripper.busy = gripper_busy_.load() != GripperBusy::Idle;
    const auto g = gripper_sender_->stats();
    in.gripper.command_failures = g.failed;
    if (g.failed > gripper_failures_logged_) {
      RCLCPP_ERROR(get_logger(), "MoveGripper failed (error %d, code %d %s); %lu failure(s) so far",
        static_cast<int>(g.last_failure.error), g.last_failure.code, g.last_failure.message.c_str(),
        static_cast<unsigned long>(g.failed));
      gripper_failures_logged_ = g.failed;
    }
  }

  StateSnapshot snap;
  if (receiver_->latest(snap)) {
    const auto age = std::chrono::steady_clock::now() - snap.received_at;
    in.frame_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
    // read() judges freshness at the controller's pace; this only has to tell up from down. Between
    // an error and the link monitor's restart of the stream the frames mean nothing.
    const bool trusted = monitoring_.load() || in.hardware != HardwareState::Unconfigured;
    const auto max_age = std::max<std::chrono::steady_clock::duration>(
      comm_timeout_, std::chrono::milliseconds(500));
    if (trusted && receiver_->running() && snap.fresh(max_age)) {
      in.link_up = true;
      in.frame = snap.state;
    }
  }

  const auto now = std::chrono::steady_clock::now();
  const bool changed = status_tracker_.update(in, std::chrono::system_clock::now());
  if (changed || now - status_published_at_ >= std::chrono::seconds(1)) {
    status_pub_->publish(status_tracker_.status());
    status_published_at_ = now;
  }
}

void FrRobotHardwareInterface::closeConnections()
{
  configured_ = false;
  sender_->stop();
  if (gripper_sender_) {gripper_sender_->stop();}
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

return_type FrRobotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // The receiver is busy this instant: keep the previous frame. With none yet (the first read after
  // configure) wait the moment a copy takes, rather than judge the empty default frame stale.
  if (!receiver_->tryLatest(frame_) && frame_.frame_count == 0) {
    receiver_->latest(frame_);
  }
  const StateSnapshot & snap = frame_;
  const bool stale = !snap.fresh(comm_timeout_);
  const bool link_lost = !receiver_->running();
  const bool sender_dead = active_.load() &&
    (!sender_->running() || sender_->commError() != ApiError::None);

  // COMM: the link cannot be trusted, so nothing in the frames can be (R3, R5)
  if (stale || link_lost || sender_dead) {
    fault_class_ = FaultClass::Comm;
    comm_reason_ = stale ? CommReason::NoFrame :
      link_lost ? CommReason::LinkLost : CommReason::ServoFailed;
    set_state(fault_class_interface_, static_cast<double>(FaultClass::Comm));
    logFault(FaultClass::Comm, stale ? "no state frame in time" :
      link_lost ? "state connection lost" : "servo stream connection failed");
    return return_type::ERROR;  // -> on_error -> UNCONFIGURED
  }

  const RobotState384 & s = snap.state;

  const Fault fault = effectiveFault(s, protective_stop_.load());
  const FaultClass fc = fault.fault_class;
  fault_class_ = fc;
  publishJoints(s);
  set_state(fault_class_interface_, static_cast<double>(fc));
  if (hasGripper()) {
    readGripper(snap, fc);
  }
  // The arm cannot move while the gripper does (sdk_quirks G9): trajectories wait meanwhile. Set
  // here, so the controllers' update in this cycle already sees a motion write() started last cycle.
  const bool gripper_busy = hasGripper() && gripper_busy_.load() != GripperBusy::Idle;
  set_state(speed_scaling_interface_, active_.load() && gripper_busy ? 0.0 : 1.0);
  if (active_.load() && fc != FaultClass::None) {
    sender_->halt();  // stop at once; write() answers DEACTIVATE
  }
  logFault(fc, fault.detail);
  return return_type::OK;
}

return_type FrRobotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {
    return return_type::OK;
  }
  if (fault_class_.load() != FaultClass::None) {
    sender_->halt();
    if (hasGripper()) {gripper_sender_->discard();}
    return return_type::DEACTIVATE;  // -> INACTIVE; the operator resets and reactivates (F3)
  }
  JointArray command;
  for (int i = 0; i < 6; ++i) {
    command[i] = get_command<double>(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION) *
      kRadToDeg;
  }
  if (hasGripper()) {
    writeGripper(gate_->commandedMoved(command));
  }
  if (!gate_->sendCycle()) {
    return return_type::OK;  // controller runs faster than cmdT: only every ratio_-th cycle is a segment
  }

  // What to do with the arm this cycle (arm_gate.hpp): the step check, and the parking while the
  // gripper moves. The firmware only limits speed/acceleration/range itself (sdk_quirks S14) and a
  // violation puts the robot in a fault (S11), so this is not optional.
  const SegmentDecision d = gate_->decide(command, hasGripper() &&
    gripper_busy_.load() != GripperBusy::Idle);
  if (d.action == Segment::Refuse) {
    RCLCPP_ERROR(get_logger(), "command refused: joint %s asks for %.3f deg after %.3f deg in one "
      "%d ms step (limit %.3f deg)", joint_names_[d.violation.joint].c_str(), d.violation.to_deg,
      d.violation.from_deg, cmd_t_ms_, d.violation.limit_deg);
    protective_stop_ = ProtectiveStop::StepLimit;
    fault_class_ = FaultClass::RobotFault;
    sender_->halt();
    return return_type::DEACTIVATE;
  }
  if (!sender_->enqueue(d.target)) {
    // full queue or dead sender: the stream to the controller is broken
    fault_class_ = FaultClass::Comm;
    comm_reason_ = CommReason::EnqueueFailed;
    return return_type::ERROR;
  }
  gate_->sent(d.target);
  return return_type::OK;
}

// ---- gripper ----

bool FrRobotHardwareInterface::gripperActive(const RobotState384 & s) const
{
  return (s.gripper_active >> (gripper_index_ - 1)) & 1u;   // gripper 1 is bit 0 (sdk_quirks G5)
}

void FrRobotHardwareInterface::readGripper(const StateSnapshot & snap, FaultClass fc)
{
  const RobotState384 & s = snap.state;
  const bool active = gripperActive(s);
  const auto state = gripper_motion_.frame(
    active, s.gripper_motiondone, s.gripper_position, std::chrono::steady_clock::now());
  if (state.outcome) {
    if (*state.outcome == GripperOutcome::TimedOut) {
      RCLCPP_ERROR(get_logger(), "gripper motion not done after 30 s; arm commands accepted again");
    }
    gripper_finished_.store((gripper_sent_id_ << 3) | static_cast<uint64_t>(*state.outcome));
    auto moving = GripperBusy::Moving;
    gripper_busy_.compare_exchange_strong(moving, GripperBusy::Idle);
  }
  set_state(gripper_joint_ + "/" + hardware_interface::HW_IF_POSITION, state.opening);
  set_state(gripper_joint_ + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  gripper_opening_ = state.opening;
  gripper_source_ = state.source;
  const bool usable = active && s.gripper_fault == 0 && fc == FaultClass::None;
  if (gripper_usable_.exchange(usable) && !usable) {
    ++gripper_epoch_;   // a goal accepted before never runs, not even once the gripper is back
  }
  // MoveGripper only runs while no ServoJ flows (G8): the stream paused and the queue drained
  const bool idle = sender_->paused() && fr_client_library::servoQueueLength(s) == 0;
  stream_idle_frames_ = idle ? stream_idle_frames_ + 1 : 0;
}

void FrRobotHardwareInterface::writeGripper(bool arm_moves)
{
  // The goal's MoveGripper waits for the motion in progress and for the arm to stand (G8, G9)
  if (!gripper_sender_->running()) {
    return;   // its thread is gone: the executor ends the goal instead
  }
  std::unique_lock<std::mutex> lock(gripper_goal_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;   // the executor holds it this instant: next cycle
  }
  if (!gripper_goals_.maySend(gripper_epoch_.load(), gripper_usable_.load(),
    gripper_busy_.load() != GripperBusy::Idle, stream_idle_frames_, arm_moves))
  {
    return;
  }
  if (gripper_sender_->post(gripper_goals_.goal().command)) {   // never blocks; busy: again next cycle
    gripper_goals_.markSent();
    gripper_motion_.sent(gripper_goals_.goal().command.position, std::chrono::steady_clock::now());
    gripper_sent_id_ = gripper_goals_.goal().id;
    gripper_busy_ = GripperBusy::Moving;
  }
}

void FrRobotHardwareInterface::onGripperGoalAccepted(const std::shared_ptr<GripperGoalHandle> & handle)
{
  const auto & c = handle->get_goal()->command;
  std::lock_guard<std::mutex> lock(gripper_goal_mutex_);
  if (const auto replaced = gripper_goals_.accept(GripperGoals::command(c.position[0],
    c.effort.empty() ? 0.0 : c.effort[0]), gripper_epoch_.load(), gripper_goal_ids_))
  {
    // the controller cannot interrupt a motion (sdk_quirks G15): the new goal's waits for it
    finishGripperGoal(GoalEnd::Aborted, "preempted by a new goal", replaced);
  }
  gripper_handle_ = handle;
  const GripperGoal & goal = gripper_goals_.goal();
  RCLCPP_INFO(get_logger(), "gripper goal %lu: %d %%, speed %d %%, force %d %%",
    static_cast<unsigned long>(goal.id), goal.command.position, goal.command.velocity,
    goal.command.force);
}

void FrRobotHardwareInterface::onGripperGoalTimer()
{
  std::lock_guard<std::mutex> lock(gripper_goal_mutex_);
  if (!gripper_handle_) {
    return;   // no goal to report on
  }
  if (gripper_handle_->is_canceling()) {
    finishGripperGoal(gripper_goals_.canceled().end, gripper_goals_.canceled().why,
      gripper_goals_.goal().id);
    return;
  }
  const GripperGoal & goal = gripper_goals_.goal();
  const auto stale = gripper_goals_.stale(gripper_epoch_.load());
  if (stale.end != GoalEnd::Running) {
    finishGripperGoal(stale.end, stale.why, goal.id);
    return;
  }
  if (goal.sent && !gripper_sender_->running()) {
    // its thread is gone: a goal waiting for the frame's outcome would never be answered
    finishGripperGoal(gripper_goals_.senderGone().end, gripper_goals_.senderGone().why, goal.id);
    return;
  }
  const GoalEnding ending = gripper_goals_.outcome(gripper_finished_.load());
  if (ending.end != GoalEnd::Running) {
    finishGripperGoal(ending.end, ending.why, goal.id, ending.reached, ending.stalled);
  }
}

void FrRobotHardwareInterface::finishGripperGoal(GoalEnd end, const std::string & why, uint64_t id,
  bool reached, bool stalled)
{
  auto result = std::make_shared<GripperAction::Result>();
  result->state.name = {gripper_joint_};
  result->state.position = {gripper_opening_.load()};
  result->reached_goal = reached;
  result->stalled = stalled;
  const auto & handle = gripper_handle_;
  if (end == GoalEnd::Succeeded) {
    handle->succeed(result);
  } else if (end == GoalEnd::Canceled) {
    handle->canceled(result);
  } else {
    handle->abort(result);
  }
  if (end == GoalEnd::Aborted) {
    RCLCPP_WARN(get_logger(), "gripper goal %lu aborted: %s", id, why.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "gripper goal %lu %s: %s", id,
      end == GoalEnd::Succeeded ? "succeeded" : "canceled", why.c_str());
  }
  gripper_handle_.reset();
  gripper_goals_.forget();
}

void FrRobotHardwareInterface::onGripperActivate(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const auto log = get_logger();
  const auto refuse = [&](const std::string & why) {
      response->success = false;
      response->message = why;
      RCLCPP_WARN(log, "gripper activation refused: %s", why.c_str());
    };
  std::lock_guard<std::mutex> lock(api_mutex_);   // its own XML-RPC connection, as reset_error
  StateSnapshot snap;
  if (!configured_.load()) {
    return refuse("hardware not configured");
  }
  receiver_->latest(snap);
  if (!snap.fresh(comm_timeout_)) {
    return refuse("no fresh state frame");
  }
  const RobotState384 & s = snap.state;
  if (const FaultClass fc = fr_client_library::classifyState(s); fc != FaultClass::None) {
    return refuse(std::string(fr_client_library::toString(fc)) + ": " +
             fr_client_library::describeFault(s));
  }
  if (const ProtectiveStop stop = protective_stop_.load(); stop != ProtectiveStop::None) {
    return refuse(toString(stop));
  }
  // A gripper fault is no reason to refuse: activating is what clears it (sdk_quirks G11)
  if (gripperActive(s)) {
    return refuse("gripper " + std::to_string(gripper_index_) + " is already active");
  }
  for (int i = 0; i < 6; ++i) {
    if (std::abs(s.actual_qd[i]) > kStillDegPerS) {
      return refuse("the arm is moving");
    }
  }
  // While ServoJ flows the controller holds gripper commands back (sdk_quirks G8)
  if (active_.load() && !sender_->paused()) {
    return refuse("the arm is being commanded (servo stream running)");
  }
  auto idle = GripperBusy::Idle;
  if (!gripper_busy_.compare_exchange_strong(idle, GripperBusy::Activating)) {
    return refuse("the gripper is moving");
  }
  // From here the arm is kept still until the activation is over (G9), and no goal from before runs
  ++gripper_epoch_;

  RCLCPP_WARN(log, "activating gripper %d: it opens to its full stroke", gripper_index_);
  bool ok = false;
  if (const auto st = api_->actGripper(gripper_index_); !st.ok()) {
    refuse("ActGripper failed (error " + std::to_string(static_cast<int>(st.error)) +
      ", code " + std::to_string(st.code) + ")");
  } else {
    const auto deadline = std::chrono::steady_clock::now() + kGripperActivationTimeout;
    while (!ok && std::chrono::steady_clock::now() < deadline) {
      ok = receiver_->latest(snap) && gripperActive(snap.state) && snap.state.gripper_fault == 0;
      if (!ok) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if (ok) {
      response->success = true;
      response->message = "gripper " + std::to_string(gripper_index_) + " active (fully open)";
      RCLCPP_INFO(log, "%s", response->message.c_str());
    } else {
      refuse("ActGripper was accepted but the gripper did not report active within 3 s");
    }
  }
  gripper_busy_ = GripperBusy::Idle;
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
