// A fake Fairino controller on 127.0.0.1: XML-RPC (20003), state stream (20004) and instant
// commands (8080) on ephemeral ports. Enough of the protocol for the hardware plugin: tracks the
// last ServoJ as the robot position, and lets a test inject faults, silence and disconnects.
#pragma once

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "fr_client_library/robot_state_384.hpp"
#include "fr_robot_driver/kinematics.hpp"

namespace fr_test
{
// Accept loop; one thread per connection; handlers return when `stop` is set or the peer is gone.
class TcpServer
{
public:
  using Handler = std::function<void(int fd, const std::atomic<bool> & stop)>;

  explicit TcpServer(Handler handler)
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    const int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    ::listen(listen_fd_, 8);
    acceptor_ = std::thread([this, handler] {
        for (;;) {
          const int fd = ::accept(listen_fd_, nullptr, nullptr);
          if (fd < 0) {return;}
          std::lock_guard<std::mutex> lock(mutex_);
          connections_.emplace_back([this, handler, fd] {
              handler(fd, stop_);
              ::close(fd);
            });
        }
      });
  }
  ~TcpServer()
  {
    stop_ = true;
    ::shutdown(listen_fd_, SHUT_RDWR);
    acceptor_.join();
    for (auto & t : connections_) {t.join();}
    ::close(listen_fd_);
  }
  uint16_t port() const {return port_;}

private:
  int listen_fd_;
  uint16_t port_;
  std::atomic<bool> stop_{false};
  std::thread acceptor_;
  std::mutex mutex_;
  std::vector<std::thread> connections_;
};

// Waits up to timeout_ms for data; returns bytes read, 0 on timeout, -1 on EOF/error.
inline ssize_t recvSome(int fd, char * buf, size_t n, int timeout_ms)
{
  pollfd p{fd, POLLIN, 0};
  if (::poll(&p, 1, timeout_ms) <= 0) {return 0;}
  const ssize_t r = ::recv(fd, buf, n, 0);
  return r > 0 ? r : -1;
}

inline bool sendAll(int fd, const std::string & s)
{
  return ::send(fd, s.data(), s.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(s.size());
}

class FakeController
{
public:
  FakeController()
  : xmlrpc_([this](int fd, const std::atomic<bool> & stop) {serveXmlRpc(fd, stop);}),
    state_([this](int fd, const std::atomic<bool> & stop) {serveState(fd, stop);}),
    instant_([this](int fd, const std::atomic<bool> & stop) {serveInstant(fd, stop);})
  {
    st_.rbtEnableState = 1;
    st_.robot_state = 1;
    st_.robot_mode = 1;
    const double home[6] = {0, -90, 90, -90, -90, 0};
    std::memcpy(st_.jt_cur_pos, home, sizeof(home));
  }

  uint16_t xmlrpcPort() const {return xmlrpc_.port();}
  uint16_t statePort() const {return state_.port();}
  uint16_t instantPort() const {return instant_.port();}

  // ---- test controls ----
  void edit(const std::function<void(fr_client_library::RobotState384 &)> & f)
  {
    std::lock_guard<std::mutex> l(m_);
    f(st_);
  }
  void setVersion(const std::string & v) {std::lock_guard<std::mutex> l(m_); version_ = v;}
  void setModel(const std::string & m) {std::lock_guard<std::mutex> l(m_); model_ = m;}
  // The reported flange pose is the FR5's FK of the joint angles, plus this error on x [mm].
  std::atomic<double> flange_error_mm{0.0};
  std::atomic<bool> silent{false};        // keep the connection, stop sending frames
  std::atomic<bool> drop_state{false};    // close the state connection

  // ---- observations ----
  std::vector<std::array<double, 6>> servoJ() const {std::lock_guard<std::mutex> l(m_); return servoj_;}
  int count(const std::string & method) const
  {
    std::lock_guard<std::mutex> l(m_);
    int n = 0;
    for (const auto & x : methods_) {n += x == method;}
    return n;
  }
  int stops() const {return stops_.load();}
  fr_client_library::RobotState384 state() const {std::lock_guard<std::mutex> l(m_); return st_;}

private:
  static std::string array(const std::string & values)
  {
    return "<value><array><data>" + values + "</data></array></value>";
  }
  static std::string i4(int v) {return "<value><i4>" + std::to_string(v) + "</i4></value>";}
  static std::string str(const std::string & s) {return "<value><string>" + s + "</string></value>";}

  void serveXmlRpc(int fd, const std::atomic<bool> & stop)
  {
    std::string in;
    char buf[2048];
    while (!stop) {
      std::size_t end;
      while ((end = in.find("\r\n\r\n")) == std::string::npos) {
        const ssize_t n = recvSome(fd, buf, sizeof(buf), 50);
        if (n < 0) {return;}
        if (n == 0 && stop) {return;}
        in.append(buf, static_cast<std::size_t>(std::max<ssize_t>(n, 0)));
      }
      const auto cl = in.find("Content-length: ");
      const std::size_t len = cl == std::string::npos ? 0 : std::stoul(in.substr(cl + 16));
      while (in.size() < end + 4 + len) {
        const ssize_t n = recvSome(fd, buf, sizeof(buf), 50);
        if (n < 0) {return;}
        if (n == 0 && stop) {return;}
        in.append(buf, static_cast<std::size_t>(std::max<ssize_t>(n, 0)));
      }
      const std::string body = in.substr(end + 4, len);
      in.erase(0, end + 4 + len);
      const auto b = body.find("<methodName>") + 12;
      const std::string method = body.substr(b, body.find("</methodName>") - b);
      const std::string value = answer(method, body);
      const std::string xml = "<?xml version=\"1.0\"?><methodResponse><params><param>" + value +
        "</param></params></methodResponse>";
      const std::string http = "HTTP/1.1 200 OK\r\nContent-type: text/xml\r\nContent-length: " +
        std::to_string(xml.size()) + "\r\nConnection: Keep-Alive\r\nKeep-Alive: timeout=15, max=30\r\n\r\n" + xml;
      if (!sendAll(fd, http)) {return;}
    }
  }

  std::string answer(const std::string & method, const std::string & body)
  {
    std::lock_guard<std::mutex> l(m_);
    methods_.push_back(method);
    if (method == "GetSoftwareVersion") {
      return array(i4(0) + str(model_) + str(version_) + str("V3.8.8-LA"));
    }
    if (method == "GetRobotRealtimeStateSamplePeriod") {
      return array(i4(0) + i4(period_));
    }
    if (method == "SetRobotRealtimeStateSamplePeriod") {
      const auto p = body.find("<i4>") + 4;
      const int v = std::stoi(body.substr(p));
      if (v < 8) {return i4(4);}  // the simulator refuses below 8 ms
      period_ = v;
      return i4(0);
    }
    if (method == "ServoJ") {
      std::array<double, 6> t{};
      auto pos = body.find("<data>");
      const auto end = body.find("</data>", pos);
      for (int i = 0; i < 6; ++i) {
        pos = body.find("<double>", pos) + 8;
        if (pos > end) {break;}
        t[i] = std::stod(body.substr(pos));
      }
      servoj_.push_back(t);
      for (int i = 0; i < 6; ++i) {st_.jt_cur_pos[i] = t[i];}  // the robot follows at once
    }
    if (method == "ResetAllError" && st_.EmergencyStop == 0) {
      st_.main_code = 0;
      st_.sub_code = 0;
      st_.collisionState = 0;
    }
    return i4(0);
  }

  void serveState(int fd, const std::atomic<bool> & stop)
  {
    uint8_t cnt = 0;
    while (!stop && !drop_state) {
      if (!silent) {
        fr_client_library::RobotState384 s;
        {
          std::lock_guard<std::mutex> l(m_);
          s = st_;
        }
        setFlange(s);
        char frame[fr_client_library::kRobotState384Size] = {};
        s.frame_head = 0x5A5A;
        s.frame_cnt = cnt++;
        s.data_len = static_cast<uint16_t>(fr_client_library::kRobotState384Size - 7);
        std::memcpy(frame, &s, sizeof(s));
        uint16_t sum = 0;
        for (std::size_t i = 0; i < sizeof(frame) - 2; ++i) {sum = static_cast<uint16_t>(sum + static_cast<uint8_t>(frame[i]));}
        std::memcpy(frame + sizeof(frame) - 2, &sum, 2);
        if (!sendAll(fd, std::string(frame, sizeof(frame)))) {return;}
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
  }

  // flange_cur_pos as the controller reports it: mm and deg, URDF rpy, base_link frame
  void setFlange(fr_client_library::RobotState384 & s) const
  {
    static const auto kin = fr_robot_driver::parseKinematics(
      "0 0 0 0 0 0; 0 0 0.152 1.5708 0 0; -0.425 0 0 0 0 0; -0.39501 0 0 0 0 0; "
      "0 0 0.1021 1.5708 0 0; 0 0 0.102 -1.5708 0 0", 0.1);
    std::array<double, 6> q;
    for (int i = 0; i < 6; ++i) {q[i] = s.jt_cur_pos[i];}
    const auto f = fr_robot_driver::flangePose(*kin, q);
    const double pitch = -std::asin(std::fmax(-1.0, std::fmin(1.0, f.R[2][0])));
    const double roll = std::atan2(f.R[2][1], f.R[2][2]);
    const double yaw = std::atan2(f.R[1][0], f.R[0][0]);
    const double v[6] = {f.p[0] * 1000.0 + flange_error_mm.load(), f.p[1] * 1000.0, f.p[2] * 1000.0,
      roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI};
    for (int i = 0; i < 6; ++i) {s.flange_cur_pos[i] = v[i];}
  }

  void serveInstant(int fd, const std::atomic<bool> & stop)
  {
    std::string in;
    char buf[256];
    while (!stop) {
      const ssize_t n = recvSome(fd, buf, sizeof(buf), 50);
      if (n < 0) {return;}
      in.append(buf, static_cast<std::size_t>(std::max<ssize_t>(n, 0)));
      std::size_t end;
      while ((end = in.find("/b/f")) != std::string::npos) {
        const std::string frame = in.substr(0, end + 4);
        in.erase(0, end + 4);
        const auto a = frame.find("III") + 3, b = frame.find("III", a), c = frame.find("III", b + 3);
        const std::string cnt = frame.substr(a, b - a), id = frame.substr(b + 3, c - b - 3);
        if (id == "102") {++stops_;}
        sendAll(fd, "/f/bIII" + cnt + "III" + id + "III1III1III/b/f");
      }
    }
  }

  mutable std::mutex m_;
  fr_client_library::RobotState384 st_{};
  std::string version_ = "v3.8.4.2";
  std::string model_ = "FR5-V1-002(V6.0)";
  int period_ = 8;
  std::vector<std::array<double, 6>> servoj_;
  std::vector<std::string> methods_;
  std::atomic<int> stops_{0};

  TcpServer xmlrpc_;   // last: their threads use the members above
  TcpServer state_;
  TcpServer instant_;
};

}  // namespace fr_test
