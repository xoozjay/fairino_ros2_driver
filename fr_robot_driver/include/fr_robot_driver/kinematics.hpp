// Forward kinematics of a Fairino arm from the fr_description parameters, to cross-check the
// state frame's flange_cur_pos against jt_cur_pos (docs/fault_handling.md section 8).
//
// Verified on the 3.8.3 simulator (FR5 V6.0, six poses): flange_cur_pos is in mm and deg, in the
// URDF base_link frame, orientation as R = Rz(rz) * Ry(ry) * Rx(rx) (URDF roll/pitch/yaw), and the
// flange is the wrist_3 frame moved by flange_offset along z. Our FK matched it within 0.25 mm and
// 0.1 deg. ❓ a robot installed other than upright may report its base frame differently.
#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <sstream>
#include <string>

namespace fr_robot_driver
{
struct JointOrigin
{
  std::array<double, 3> xyz{};  // m, relative to the previous joint frame
  std::array<double, 3> rpy{};  // rad
};

struct Kinematics
{
  std::array<JointOrigin, 6> joints;  // every joint turns about its own z axis
  double flange_offset = 0;           // m along z of the last joint frame
};

// "x y z r p y; ..." six times, the form xacro passes from default_kinematics.yaml.
inline std::optional<Kinematics> parseKinematics(const std::string & joints, double flange_offset)
{
  Kinematics k;
  k.flange_offset = flange_offset;
  std::istringstream all(joints);
  std::string item;
  int n = 0;
  while (std::getline(all, item, ';')) {
    if (n >= 6) {
      return std::nullopt;
    }
    std::istringstream one(item);
    double v[6];
    for (double & x : v) {
      if (!(one >> x)) {
        return std::nullopt;
      }
    }
    std::string rest;
    if (one >> rest) {
      return std::nullopt;
    }
    k.joints[n] = {{v[0], v[1], v[2]}, {v[3], v[4], v[5]}};
    ++n;
  }
  return n == 6 ? std::optional<Kinematics>(k) : std::nullopt;
}

struct Transform
{
  double R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double p[3] = {0, 0, 0};
};

inline Transform operator*(const Transform & a, const Transform & b)
{
  Transform c;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      c.R[i][j] = a.R[i][0] * b.R[0][j] + a.R[i][1] * b.R[1][j] + a.R[i][2] * b.R[2][j];
    }
    c.p[i] = a.R[i][0] * b.p[0] + a.R[i][1] * b.p[1] + a.R[i][2] * b.p[2] + a.p[i];
  }
  return c;
}

// R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in rad.
inline Transform fromRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  Transform t;
  const double R[3][3] = {
    {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
    {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
    {-sp, cp * sr, cp * cr}};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      t.R[i][j] = R[i][j];
    }
  }
  return t;
}

inline Transform rotZ(double a) {return fromRpy(0, 0, a);}

// Flange pose in the base frame for joint angles in degrees.
inline Transform flangePose(const Kinematics & k, const std::array<double, 6> & q_deg)
{
  constexpr double kDegToRad = M_PI / 180.0;
  Transform t;
  for (int i = 0; i < 6; ++i) {
    Transform origin = fromRpy(k.joints[i].rpy[0], k.joints[i].rpy[1], k.joints[i].rpy[2]);
    for (int a = 0; a < 3; ++a) {
      origin.p[a] = k.joints[i].xyz[a];
    }
    t = t * origin * rotZ(q_deg[i] * kDegToRad);
  }
  Transform flange;
  flange.p[2] = k.flange_offset;
  return t * flange;
}

struct FlangeMismatch
{
  double position_mm = 0;  // distance between the two flange positions
  double angle_deg = 0;    // angle of the rotation that turns one orientation into the other
};

// `vendor` is the state frame's flange_cur_pos: x y z in mm, rx ry rz in deg. Orientations are
// compared as rotations, not as Euler angles (which wrap at +-180 and are ambiguous at pitch +-90).
inline FlangeMismatch compareFlange(
  const Kinematics & k, const std::array<double, 6> & q_deg, const double vendor[6])
{
  constexpr double kDegToRad = M_PI / 180.0;
  const Transform ours = flangePose(k, q_deg);
  const Transform theirs = fromRpy(vendor[3] * kDegToRad, vendor[4] * kDegToRad, vendor[5] * kDegToRad);

  FlangeMismatch m;
  double d2 = 0;
  for (int i = 0; i < 3; ++i) {
    const double d = ours.p[i] * 1000.0 - vendor[i];
    d2 += d * d;
  }
  m.position_mm = std::sqrt(d2);

  double trace = 0;  // trace(ours^T * theirs)
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      trace += ours.R[j][i] * theirs.R[j][i];
    }
  }
  m.angle_deg = std::acos(std::fmax(-1.0, std::fmin(1.0, (trace - 1.0) / 2.0))) / kDegToRad;
  return m;
}

}  // namespace fr_robot_driver
