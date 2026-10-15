#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "fr_client_library/http_connection.hpp"
#include "test_support.hpp"

using namespace fr_client_library;
using namespace fr_test;
using namespace std::chrono_literals;

namespace
{
HttpOptions fast()
{
  HttpOptions o;
  o.connect_timeout_ms = 500;
  o.request_timeout_ms = 300;
  return o;
}
}  // namespace

TEST(Http, RequestHeadMatchesSdkAndBodyRoundTrips)
{
  std::string head;
  std::string got_body;
  FakeServer s([&](int fd) {
      got_body = readHttpRequest(fd, &head);
      writeAll(fd, httpResponse("<pong/>"));
    });
  HttpConnection c("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(c.connect());
  const auto r = c.post("<ping/>");
  EXPECT_EQ(r.status, HttpStatus::Ok);
  EXPECT_EQ(r.body, "<pong/>");
  c.close();
  EXPECT_EQ(got_body, "<ping/>");
  EXPECT_EQ(
    head,
    "POST /RPC2 HTTP/1.1\r\nUser-Agent: XMLRPC++ 0.7\r\nHost: 127.0.0.1:" + std::to_string(s.port()) +
    "\r\nContent-Type: text/xml\r\nContent-length: 7");
}

TEST(Http, KeepAliveReusesTheConnection)
{
  std::atomic<int> connections{0};
  FakeServer s([&](int fd) {
      ++connections;
      for (int i = 0; i < 3; ++i) {
        if (readHttpRequest(fd).empty()) {return;}
        writeAll(fd, httpResponse("ok"));
      }
    });
  HttpConnection c("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(c.connect());
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(c.post("x").status, HttpStatus::Ok);
  }
  EXPECT_EQ(connections, 1);
}

TEST(Http, ResponseSplitIntoTinyPieces)
{
  FakeServer s([&](int fd) {
      readHttpRequest(fd);
      const std::string r = httpResponse("<hello/>");
      for (std::size_t i = 0; i < r.size(); i += 7) {
        writeAll(fd, r.substr(i, 7));
        std::this_thread::sleep_for(1ms);
      }
    });
  HttpConnection c("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(c.connect());
  const auto r = c.post("x");
  EXPECT_EQ(r.status, HttpStatus::Ok);
  EXPECT_EQ(r.body, "<hello/>");
}

// The simulator answers its 30th request with "Connection: close" (no Keep-Alive header).
TEST(Http, ConnectionCloseRotatesBeforeNextRequest)
{
  std::atomic<int> connections{0};
  FakeServer s([&](int fd) {
      ++connections;
      if (readHttpRequest(fd).empty()) {return;}
      writeAll(fd, httpResponse("one", "close", ""));
      // server drops the connection after the response
    });
  HttpConnection c("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(c.connect());
  EXPECT_EQ(c.post("a").status, HttpStatus::Ok);
  EXPECT_FALSE(c.isOpen());
  const auto r = c.post("b");  // transparently on a new connection
  EXPECT_EQ(r.status, HttpStatus::Ok);
  EXPECT_EQ(connections, 2);
}

// Server idle timeout is 15 s; a connection idle for nearly that long is replaced up front.
TEST(Http, IdleConnectionIsRotatedBeforeSending)
{
  std::atomic<int> connections{0};
  FakeServer s([&](int fd) {
      ++connections;
      while (!readHttpRequest(fd).empty()) {
        writeAll(fd, httpResponse("ok", "Keep-Alive", "timeout=1, max=30"));
      }
    });
  HttpConnection c("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(c.connect());
  EXPECT_EQ(c.post("a").status, HttpStatus::Ok);
  EXPECT_EQ(c.post("b").status, HttpStatus::Ok);  // timeout 1 s is within the 2 s margin
  EXPECT_EQ(connections, 2);
}

TEST(Http, ErrorClosesAndDoesNotReconnectSilently)
{
  FakeServer s([&](int fd) {readHttpRequest(fd);});  // closes without answering
  HttpConnection c("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(c.connect());
  EXPECT_EQ(c.post("a").status, HttpStatus::IoError);
  EXPECT_FALSE(c.isOpen());
  EXPECT_EQ(c.post("a").status, HttpStatus::NotConnected);  // the caller decides to reconnect
}

TEST(Http, NonOkStatusAndMissingContentLengthAreProtocolErrors)
{
  int n = 0;
  FakeServer s([&](int fd) {
      readHttpRequest(fd);
      writeAll(fd, n++ == 0 ? "HTTP/1.1 500 Oops\r\nContent-length: 0\r\n\r\n" :
        "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nbody");
    });
  HttpConnection a("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(a.connect());
  EXPECT_EQ(a.post("x").status, HttpStatus::ProtocolError);
  HttpConnection b("127.0.0.1", s.port(), fast());
  ASSERT_TRUE(b.connect());
  EXPECT_EQ(b.post("x").status, HttpStatus::ProtocolError);
}

TEST(Http, ConnectRefused)
{
  uint16_t port;
  {
    FakeServer s([](int) {});
    port = s.port();
  }
  HttpConnection c("127.0.0.1", port, fast());
  EXPECT_FALSE(c.connect());
  EXPECT_EQ(c.post("x").status, HttpStatus::NotConnected);
}
