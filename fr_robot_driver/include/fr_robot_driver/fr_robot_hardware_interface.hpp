// ros2_control SystemInterface for one Fairino robot (software version 3.8.4, see AGENTS.md).
//
// Follows docs/fault_handling.md: read()/write() never touch the network; faults stop the stream
// and are reported, never cleared automatically. Parameters come from the <hardware> tag.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "fr_client_library/fault_state.hpp"
#include "fr_client_library/instant_command.hpp"
#include "fr_client_library/robot_api.hpp"
#include "fr_client_library/robot_model.hpp"
#include "fr_client_library/servo_sender.hpp"
#include "fr_client_library/state_receiver.hpp"
#include "fr_robot_driver/kinematics.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fr_robot_driver
{
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
  void publishStatus(const fr_client_library::RobotState384 & s, fr_client_library::FaultClass fc);
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
  void onResetRequest(
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
  std::string status_prefix_;     // "<tf_prefix>fr_status/"
  std::vector<std::string> joint_names_;
  std::vector<double> max_step_deg_;  // per joint, per segment: URDF velocity limit * cmdT

  // ---- connections: created once in on_init and never replaced, only (dis)connected by the
  // lifecycle callbacks, so that the reset service (another thread) may use them at any
  // time ----
  std::mutex api_mutex_;  // api_ is shared by on_configure and the services
  std::unique_ptr<fr_client_library::RobotApi> api_;
  std::unique_ptr<fr_client_library::StateReceiver> receiver_;
  fr_client_library::SoftwareVersion version_ = fr_client_library::kTargetVersion;  // last configure's
  std::unique_ptr<fr_client_library::ServoSender> sender_;
  std::unique_ptr<fr_client_library::InstantCommandChannel> instant_;

  // ---- run state ----
  std::atomic<bool> configured_{false};
  std::atomic<bool> active_{false};
  std::atomic<fr_client_library::FaultClass> fault_class_{fr_client_library::FaultClass::None};
  std::atomic<bool> step_fault_{false};  // a command was refused by the step check; cleared on activation
  std::thread cleanup_thread_;

  fr_client_library::FaultClass logged_class_ = fr_client_library::FaultClass::None;
  fr_client_library::StateSnapshot frame_;  // read()'s newest frame (realtime thread only)
  JointArray last_cmd_deg_{};
  unsigned long write_count_ = 0;
  std::chrono::steady_clock::duration comm_timeout_{};

  // ---- reset service (lives on the controller manager's executor) ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Executor::WeakPtr executor_;
};

}  // namespace fr_robot_driver
