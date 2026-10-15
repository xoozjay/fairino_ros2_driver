// ros2_control SystemInterface for one Fairino robot (software version 3.8.4, see AGENTS.md).
//
// Follows docs/fault_handling.md: read()/write() never touch the network; faults stop the stream
// and are reported, never cleared automatically. Parameters come from the <hardware> tag.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "fr_client_library/fault_state.hpp"
#include "fr_client_library/gripper_sender.hpp"
#include "fr_client_library/instant_command.hpp"
#include "fr_client_library/robot_api.hpp"
#include "fr_client_library/robot_model.hpp"
#include "fr_client_library/servo_sender.hpp"
#include "fr_client_library/state_receiver.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "fr_robot_driver/gripper_motion.hpp"
#include "fr_robot_driver/kinematics.hpp"
#include "fr_robot_driver/status_report.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fr_robot_driver
{
enum class GripperBusy { Idle, Moving, Activating };

class FrRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ~FrRobotHardwareInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using JointArray = fr_client_library::JointArray;

  std::string param(const std::string & key, const std::string & fallback) const;
  void publishJoints(const fr_client_library::RobotState384 & s);
  void logFault(fr_client_library::FaultClass fc, const std::string & detail);
  // Does the state frame's flange_cur_pos agree with our FK of its jt_cur_pos? Wrong robot model,
  // wrong zero offsets or sign conventions show up here (docs/fault_handling.md section 8).
  bool flangeConsistent(const fr_client_library::RobotState384 & s, std::string & why) const;
  void closeConnections();
  // on_deactivate/on_error run inside ResourceManager::write()/read(), i.e. on the realtime thread:
  // they must not block, so the network part of shutting down happens on this short-lived thread.
  void startCleanup(bool send_stop, bool close_all);
  void joinCleanup();
  // The tail of the cleanup thread after on_error: while the hardware is unconfigured, restarts the
  // read-only state stream (20004) every 2 s while it is down, so that `~/status` shows what the
  // controller reports once it is back. Never touches 20003, 8080 or ServoJ. Ends when
  // joinCleanup() asks it to, and stops the stream again.
  void monitorLink();
  void publishDiagnostics();
  void setHardwareState(HardwareState s) {hardware_state_ = s;}
  void onResetRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ---- gripper (sdk_quirks G1-G7) ----
  bool hasGripper() const {return !gripper_joint_.empty();}
  bool gripperActive(const fr_client_library::RobotState384 & s) const;
  void readGripper(const fr_client_library::StateSnapshot & snap, fr_client_library::FaultClass fc);
  void writeGripper(bool arm_moves);
  void onGripperActivate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ---- parameters (from <hardware>) ----
  std::string ip_;
  uint16_t xmlrpc_port_ = 20003;
  uint16_t state_port_ = 20004;
  uint16_t instant_port_ = 8080;
  int cmd_t_ms_ = 8;
  int prefill_ = 3;
  int rt_priority_ = 0;  // SCHED_FIFO priority of the servo sender thread; 0 = off
  int state_period_ms_ = 8;
  int comm_timeout_periods_ = 3;
  int ratio_ = 1;                 // controller cycles per ServoJ segment
  bool allow_unverified_version_ = false;
  std::string model_;             // fr_description model this URDF was built for
  bool allow_unknown_model_ = false;
  bool fk_check_ = true;
  double fk_position_tolerance_mm_ = 10.0;
  double fk_angle_tolerance_deg_ = 1.0;
  Kinematics kinematics_;
  std::string fault_class_interface_;  // "<tf_prefix>fr_status/fault_class"
  std::string speed_scaling_interface_;  // "<tf_prefix>speed_scaling/speed_scaling_factor"
  std::vector<std::string> joint_names_;
  std::vector<double> max_step_deg_;  // per joint, per segment: URDF velocity limit * cmdT

  // ---- connections: created once in on_init and never replaced, only (dis)connected by the
  // lifecycle callbacks, so that the status timer and the services (other threads) may use them
  // at any time ----
  std::mutex api_mutex_;  // api_ is shared by on_configure and the services
  std::unique_ptr<fr_client_library::RobotApi> api_;
  std::unique_ptr<fr_client_library::StateReceiver> receiver_;
  fr_client_library::SoftwareVersion version_ = fr_client_library::kTargetVersion;  // last configure's
  std::unique_ptr<fr_client_library::ServoSender> sender_;
  std::unique_ptr<fr_client_library::InstantCommandChannel> instant_;
  std::atomic<bool> monitoring_{false};  // monitorLink() runs: receiver_'s frames come from it

  // ---- run state ----
  std::atomic<bool> configured_{false};
  std::atomic<bool> active_{false};
  std::atomic<fr_client_library::FaultClass> fault_class_{fr_client_library::FaultClass::None};
  std::atomic<ProtectiveStop> protective_stop_{ProtectiveStop::None};  // cleared on activation
  std::thread cleanup_thread_;
  std::mutex stop_mutex_;                // with stop_cv_: wakes monitorLink() when joinCleanup() asks to end
  std::condition_variable stop_cv_;
  bool cleanup_stop_ = false;            // guarded by stop_mutex_
  std::atomic<HardwareState> hardware_state_{HardwareState::Unconfigured};
  std::atomic<CommReason> comm_reason_{CommReason::None};  // set by read()/write(), cleared by configure/cleanup

  fr_client_library::FaultClass logged_class_ = fr_client_library::FaultClass::None;
  fr_client_library::StateSnapshot frame_;  // read()'s newest frame (realtime thread only)
  bool holding_ramp_ = false;              // smoothly ramping down to a JTC hold position
  JointArray hold_target_{};
  JointArray last_cmd_deg_{};
  unsigned long write_count_ = 0;
  std::chrono::steady_clock::duration comm_timeout_{};

  // ---- gripper (optional: the gripper_joint parameter names its joint) ----
  std::string gripper_joint_;
  int gripper_index_ = 1;
  std::unique_ptr<fr_client_library::GripperSender> gripper_sender_;
  std::atomic<bool> gripper_usable_{false};    // active, no gripper fault, no robot fault (read())
  std::atomic<GripperSource> gripper_source_{GripperSource::Frame};
  // Moving: set by write() when a MoveGripper goes out, cleared by read() once the frame shows it
  // done. Activating: set and cleared by the activation service. Either way the arm stays still
  // (sdk_quirks G9) and the next gripper command waits.
  std::atomic<GripperBusy> gripper_busy_{GripperBusy::Idle};
  GripperMotion gripper_motion_;           // realtime thread only: completion and opening inference
  int stream_idle_frames_ = 0;             // consecutive reads: stream paused, firmware queue empty
  std::atomic<double> gripper_opening_{0.0};   // the opening last reported [m], for goal results

  // ---- gripper action `<ns>/gripper_controller/gripper_cmd` (on the node below) ----
  // One goal at a time. The executor thread accepts, preempts, cancels and finishes goals; the
  // realtime thread sends a goal's MoveGripper and reports how the frame shows its motion ending.
  using GripperAction = control_msgs::action::ParallelGripperCommand;
  using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;
  struct GripperGoal
  {
    std::shared_ptr<GripperGoalHandle> handle;   // none: no goal
    uint64_t id = 0;
    uint64_t epoch = 0;                          // gripper_epoch_ when it was accepted
    fr_client_library::GripperCommand command;
    bool sent = false;                           // its MoveGripper went out
  };
  void onGripperGoalAccepted(const std::shared_ptr<GripperGoalHandle> & handle);
  void onGripperGoalTimer();
  // executor thread, gripper_goal_mutex_ held: ends the goal and forgets it
  enum class GoalEnd { Succeeded, Aborted, Canceled };
  void finishGripperGoal(GoalEnd end, const std::string & why, bool reached = false, bool stalled = false);
  std::mutex gripper_goal_mutex_;   // guards gripper_goal_; the realtime thread only try_locks it
  GripperGoal gripper_goal_;
  uint64_t gripper_goal_ids_ = 0;   // executor thread only
  // Bumped whenever goals accepted before must not run any more: activation and deactivation of
  // the hardware, an error, the gripper becoming unusable, an activation of the gripper.
  std::atomic<uint64_t> gripper_epoch_{0};
  // The goal whose motion the frame showed ending, and how: (id << 3) | GripperOutcome.
  std::atomic<uint64_t> gripper_finished_{0};
  uint64_t gripper_sent_id_ = 0;    // realtime thread only: the goal of the motion in progress
  rclcpp_action::Server<GripperAction>::SharedPtr gripper_action_;
  rclcpp::TimerBase::SharedPtr gripper_goal_timer_;

  // ---- reset service (lives on the controller manager's executor) ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  // `~/gripper/activate`, in its own callback group: it waits up to 3 s for the activation
  rclcpp::CallbackGroup::SharedPtr gripper_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_activate_service_;
  rclcpp::Executor::WeakPtr executor_;

  // ---- status topic `~/status` (same node; read()/write() never publish) ----
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  StatusTracker status_tracker_;  // only touched by the timer callback
  uint64_t gripper_failures_logged_ = 0;  // likewise
  std::chrono::steady_clock::time_point status_published_at_{};
};

}  // namespace fr_robot_driver
