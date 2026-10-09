// State frame of Fairino software version 3.8.4 (TCP 20004).
//
// Layout copied from fairino-cpp-sdk v2.2.4-robot3.8.4 (robot_types.h,
// ROBOT_STATE_PKG), Apache-2.0, (c) FAIR-INNOVATION. Field order and types are
// unchanged; names and nested type names are kept so it can be diffed against
// the SDK. Packed, little-endian host assumed.
#pragma once

#include <cstddef>
#include <cstdint>

namespace fr_client_library
{
#pragma pack(push, 1)

struct RobotTime
{
  uint16_t year;
  uint8_t mouth;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
};

struct RobotAuxState  // SDK: robot_aux_state
{
  uint8_t servoId;
  int32_t servoErrCode;
  int32_t servoState;
  double servoPos;
  float servoVel;
  float servoTorque;
};

struct ExtAxisStatus  // SDK: EXT_AXIS_STATUS
{
  double pos;
  double vel;
  int32_t errorCode;
  uint8_t ready;
  uint8_t inPos;
  uint8_t alarm;
  uint8_t flerr;
  uint8_t nlimit;
  uint8_t pLimit;
  uint8_t mdbsOffLine;
  uint8_t mdbsTimeout;
  uint8_t homingStatus;
};

struct WeldingBreakoffState  // SDK: WELDING_BREAKOFF_STATE
{
  uint8_t breakOffState;
  uint8_t weldArcState;
};

struct RobotState384  // SDK: ROBOT_STATE_PKG
{
  uint16_t frame_head;
  uint8_t frame_cnt;
  uint16_t data_len;
  uint8_t program_state;  // 1 stop, 2 run, 3 pause
  uint8_t robot_state;    // 1 stop, 2 run, 3 pause, 4 drag
  int32_t main_code;
  int32_t sub_code;
  uint8_t robot_mode;  // 0 auto, 1 manual
  double jt_cur_pos[6];
  double tl_cur_pos[6];
  double flange_cur_pos[6];
  double actual_qd[6];
  double actual_qdd[6];
  double target_TCP_CmpSpeed[2];
  double target_TCP_Speed[6];
  double actual_TCP_CmpSpeed[2];
  double actual_TCP_Speed[6];
  double jt_cur_tor[6];
  int32_t tool;
  int32_t user;
  uint8_t cl_dgt_output_h;
  uint8_t cl_dgt_output_l;
  uint8_t tl_dgt_output_l;
  uint8_t cl_dgt_input_h;
  uint8_t cl_dgt_input_l;
  uint8_t tl_dgt_input_l;
  uint16_t cl_analog_input[2];
  uint16_t tl_anglog_input;
  double ft_sensor_raw_data[6];
  double ft_sensor_data[6];
  uint8_t ft_sensor_active;
  uint8_t EmergencyStop;
  int32_t motion_done;
  uint8_t gripper_motiondone;
  int32_t mc_queue_len;
  uint8_t collisionState;  // 1 collision
  int32_t trajectory_pnum;
  uint8_t safety_stop0_state;  // SI0
  uint8_t safety_stop1_state;  // SI1
  uint8_t gripper_fault_id;
  uint16_t gripper_fault;
  uint16_t gripper_active;
  uint8_t gripper_position;
  int8_t gripper_speed;
  int8_t gripper_current;
  int32_t gripper_temp;
  int32_t gripper_voltage;
  RobotAuxState aux_state;
  ExtAxisStatus extAxisStatus[4];
  uint16_t extDIState[8];
  uint16_t extDOState[8];
  uint16_t extAIState[4];
  uint16_t extAOState[4];
  int32_t rbtEnableState;
  double jointDriverTorque[6];
  double jointDriverTemperature[6];
  RobotTime robotTime;
  int32_t softwareUpgradeState;
  uint16_t endLuaErrCode;
  uint16_t cl_analog_output[2];
  uint16_t tl_analog_output;
  float gripperRotNum;
  uint8_t gripperRotSpeed;
  uint8_t gripperRotTorque;
  WeldingBreakoffState weldingBreakOffState;
  double jt_tgt_tor[6];
  int32_t smartToolState;
  float wideVoltageCtrlBoxTemp;
  uint16_t wideVoltageCtrlBoxFanCurrent;
  uint16_t check_sum;
};

#pragma pack(pop)

// Frame = head(2) cnt(1) data_len(2) payload(data_len) checksum(2) => data_len + 7.
constexpr std::size_t kFrameOverhead = 7;
constexpr std::size_t kRobotState384Size = 960;

static_assert(sizeof(RobotState384) == kRobotState384Size, "3.8.4 state frame is 960 bytes");
static_assert(offsetof(RobotState384, data_len) == 3, "");
static_assert(offsetof(RobotState384, robot_state) == 6, "");
static_assert(offsetof(RobotState384, main_code) == 7, "");
static_assert(offsetof(RobotState384, sub_code) == 11, "");
static_assert(offsetof(RobotState384, robot_mode) == 15, "");
static_assert(offsetof(RobotState384, EmergencyStop) == 549, "");
static_assert(offsetof(RobotState384, collisionState) == 559, "");
static_assert(offsetof(RobotState384, safety_stop0_state) == 564, "");
static_assert(offsetof(RobotState384, safety_stop1_state) == 565, "");
static_assert(offsetof(RobotState384, rbtEnableState) == 771, "");
static_assert(offsetof(RobotState384, check_sum) == kRobotState384Size - 2, "");

}  // namespace fr_client_library
