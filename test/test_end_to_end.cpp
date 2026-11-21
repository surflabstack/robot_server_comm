// Copyright 2026 Anas Derkaoui
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// End to end tests over a real TCP connection on the loopback interface:
// DataTransmitter + TcpTransport on one side, FrameServer + FramePrinter on the
// other. Only ROS is left out, everything the bytes go through is the real code.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "robot_server_comm/core/data_transmitter.hpp"
#include "robot_server_comm/server/frame_printer.hpp"
#include "robot_server_comm/server/frame_server.hpp"
#include "robot_server_comm/transport/tcp_transport.hpp"
#include "test_helpers.hpp"

namespace robot_server_comm
{

using std::chrono::milliseconds;
using test_support::wait_until;

namespace
{

/// Runs a FrameServer in its own thread and records everything it reports.
class ServerRunner
{
public:
  explicit ServerRunner(
    std::uint16_t port = 0, milliseconds client_timeout = milliseconds(5000))
  : printer_(output_, FramePrinterConfig{milliseconds(0), milliseconds(0)})
  {
    FrameServerConfig config;
    config.port = port;
    config.client_timeout = client_timeout;

    FrameServerHandlers handlers;
    handlers.on_connect = [this](const ClientInfo & client) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++connections_;
        printer_.on_connect(client);
      };
    handlers.on_frame = [this](const ClientInfo & client, const Frame & frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frame.type == FrameType::kData) {
          data_.push_back(frame);
          robots_.push_back(client.robot_id);
        }
        printer_.on_frame(client, frame);
      };
    handlers.on_disconnect = [this](const ClientInfo & client, const std::string & reason) {
        std::lock_guard<std::mutex> lock(mutex_);
        disconnect_reasons_.push_back(reason);
        printer_.on_disconnect(client, reason);
      };

    server_ = std::make_unique<FrameServer>(config, handlers);
    std::string error;
    started_ = server_->start(error);
    start_error_ = error;
    port_ = server_->port();
    thread_ = std::thread([this] {
          while (!stop_.load()) {
            server_->poll_once(milliseconds(10));
          }
        });
  }

  ~ServerRunner()
  {
    stop_.store(true);
    thread_.join();
    server_->stop();
  }

  bool started() const {return started_;}
  const std::string & start_error() const {return start_error_;}
  std::uint16_t port() const {return port_;}

  std::vector<Frame> data()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  std::size_t data_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.size();
  }

  std::vector<std::string> robots()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return robots_;
  }

  std::size_t connections()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return connections_;
  }

  std::vector<std::string> disconnect_reasons()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return disconnect_reasons_;
  }

  std::string output()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_.str();
  }

  ReceptionTotals totals()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return printer_.totals();
  }

private:
  std::mutex mutex_;
  std::ostringstream output_;
  FramePrinter printer_;
  std::unique_ptr<FrameServer> server_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  bool started_{false};
  std::string start_error_;
  std::uint16_t port_{0};
  std::vector<Frame> data_;
  std::vector<std::string> robots_;
  std::vector<std::string> disconnect_reasons_;
  std::size_t connections_{0};
};

std::unique_ptr<DataTransmitter> make_robot(std::uint16_t port, const std::string & robot_id)
{
  TcpTransportConfig transport;
  transport.host = "127.0.0.1";
  transport.port = port;
  transport.connect_timeout = milliseconds(500);
  transport.send_timeout = milliseconds(2000);

  TransmitterConfig config;
  config.robot_id = robot_id;
  config.max_queue_messages = 1000;
  config.reconnect_initial_delay = milliseconds(20);
  config.reconnect_max_delay = milliseconds(100);
  config.heartbeat_period = milliseconds(100);

  auto transmitter = std::make_unique<DataTransmitter>(
    std::make_unique<TcpTransport>(transport), config, [](LogLevel, const std::string &) {});
  transmitter->start();
  return transmitter;
}

std::vector<std::uint8_t> payload_of(std::size_t size, std::uint64_t index)
{
  std::vector<std::uint8_t> payload(size);
  for (std::size_t i = 0; i < size; ++i) {
    payload[i] = static_cast<std::uint8_t>(index * 13 + i);
  }
  return payload;
}

}  // namespace

TEST(EndToEnd, SensorStreamsReachTheServerCompleteAndInOrder)
{
  ServerRunner server;
  ASSERT_TRUE(server.started()) << server.start_error();
  auto robot = make_robot(server.port(), "e2e_robot");

  // Sizes close to the real sensors: IMU (324 B), GNSS (125 B), LiDAR scan (about 1 MB).
  constexpr std::uint64_t kRounds = 40;
  for (std::uint64_t i = 0; i < kRounds; ++i) {
    const auto imu = payload_of(324, i);
    const auto fix = payload_of(125, i);
    const auto cloud = payload_of(1000000, i);
    ASSERT_TRUE(robot->submit("/imu/data", "sensor_msgs/msg/Imu", imu.data(), imu.size()));
    ASSERT_TRUE(
      robot->submit("/ublox_node/fix", "sensor_msgs/msg/NavSatFix", fix.data(), fix.size()));
    ASSERT_TRUE(
      robot->submit(
        "/velodyne_points", "sensor_msgs/msg/PointCloud2", cloud.data(), cloud.size()));
    // Leave the queue time to drain: this test is about integrity, not about overload.
    ASSERT_TRUE(
      wait_until([&robot] {return robot->stats().queued_messages < 100;}, milliseconds(10000)));
  }

  ASSERT_TRUE(
    wait_until(
      [&server, kRounds] {return server.data_count() == 3 * kRounds;}, milliseconds(20000)));

  std::uint64_t next_imu = 0;
  std::uint64_t next_cloud = 0;
  for (const Frame & frame : server.data()) {
    if (frame.topic == "/imu/data") {
      EXPECT_EQ(next_imu, frame.sequence);
      EXPECT_EQ(payload_of(324, next_imu), frame.payload);
      ++next_imu;
    } else if (frame.topic == "/velodyne_points") {
      EXPECT_EQ(next_cloud, frame.sequence);
      EXPECT_EQ("sensor_msgs/msg/PointCloud2", frame.message_type);
      EXPECT_EQ(payload_of(1000000, next_cloud), frame.payload);
      ++next_cloud;
    }
  }
  EXPECT_EQ(kRounds, next_imu);
  EXPECT_EQ(kRounds, next_cloud);

  const ReceptionTotals totals = server.totals();
  EXPECT_EQ(3 * kRounds, totals.data_frames);
  EXPECT_EQ(0u, totals.lost_messages);

  const std::string output = server.output();
  EXPECT_NE(std::string::npos, output.find("[HELLO] client #1 is robot 'e2e_robot'")) << output;
  EXPECT_NE(std::string::npos, output.find("[DATA] robot=e2e_robot topic=/imu/data"));
  EXPECT_NE(std::string::npos, output.find("type=sensor_msgs/msg/PointCloud2"));

  const TransmitterStats stats = robot->stats();
  EXPECT_EQ(3 * kRounds, stats.sent);
  EXPECT_EQ(0u, stats.dropped_queue_full);
  EXPECT_EQ(1u, stats.connections);
}

TEST(EndToEnd, RobotReconnectsWhenTheServerRestartsAndLosesNothingQueued)
{
  std::uint16_t port = 0;
  std::unique_ptr<DataTransmitter> robot;
  {
    ServerRunner first_server;
    ASSERT_TRUE(first_server.started()) << first_server.start_error();
    port = first_server.port();
    robot = make_robot(port, "restart_robot");

    for (std::uint64_t i = 0; i < 50; ++i) {
      const auto data = payload_of(200, i);
      ASSERT_TRUE(robot->submit("/imu/data", "sensor_msgs/msg/Imu", data.data(), data.size()));
    }
    ASSERT_TRUE(wait_until([&first_server] {return first_server.data_count() == 50;}));
  }  // the server goes down here

  ASSERT_TRUE(wait_until([&robot] {return !robot->stats().connected;}));

  // The robot keeps producing data while the server is away.
  for (std::uint64_t i = 50; i < 100; ++i) {
    const auto data = payload_of(200, i);
    ASSERT_TRUE(robot->submit("/imu/data", "sensor_msgs/msg/Imu", data.data(), data.size()));
  }
  EXPECT_EQ(50u, robot->stats().sent);

  ServerRunner second_server(port);
  ASSERT_TRUE(second_server.started()) << second_server.start_error();
  ASSERT_TRUE(
    wait_until([&second_server] {return second_server.data_count() == 50;}, milliseconds(10000)));

  const std::vector<Frame> data = second_server.data();
  for (std::size_t i = 0; i < data.size(); ++i) {
    EXPECT_EQ(50u + i, data[i].sequence);
    EXPECT_EQ(payload_of(200, 50 + i), data[i].payload);
  }
  EXPECT_EQ(0u, second_server.totals().lost_messages);
  EXPECT_EQ(2u, robot->stats().connections);
}

TEST(EndToEnd, ServerDetectsSequenceGaps)
{
  std::ostringstream output;
  FramePrinter printer(output, FramePrinterConfig{milliseconds(0), milliseconds(0)});
  ClientInfo client;
  client.id = 1;
  client.robot_id = "gap_robot";

  Frame frame;
  frame.type = FrameType::kData;
  frame.topic = "/imu/data";
  frame.message_type = "sensor_msgs/msg/Imu";
  for (const std::uint64_t sequence : {0u, 1u, 2u, 10u, 11u}) {
    frame.sequence = sequence;
    printer.on_frame(client, frame);
  }
  // Sequences 3 to 9 never arrived: 7 messages were dropped on the robot side.
  EXPECT_EQ(7u, printer.totals().lost_messages);
  EXPECT_NE(std::string::npos, output.str().find("LOST_BEFORE=7"));

  // The robot process restarts and counts from zero again: this is not a loss.
  frame.sequence = 0;
  printer.on_frame(client, frame);
  EXPECT_EQ(7u, printer.totals().lost_messages);
  EXPECT_EQ(6u, printer.totals().data_frames);
}

TEST(EndToEnd, ServerHandlesSeveralRobotsAtOnce)
{
  ServerRunner server;
  ASSERT_TRUE(server.started()) << server.start_error();
  auto alpha = make_robot(server.port(), "alpha");
  auto beta = make_robot(server.port(), "beta");

  for (std::uint64_t i = 0; i < 30; ++i) {
    const auto data = payload_of(100, i);
    ASSERT_TRUE(alpha->submit("/imu/data", "sensor_msgs/msg/Imu", data.data(), data.size()));
    ASSERT_TRUE(beta->submit("/imu/data", "sensor_msgs/msg/Imu", data.data(), data.size()));
  }
  ASSERT_TRUE(wait_until([&server] {return server.data_count() == 60;}));

  std::size_t from_alpha = 0;
  std::size_t from_beta = 0;
  for (const std::string & robot : server.robots()) {
    from_alpha += robot == "alpha" ? 1 : 0;
    from_beta += robot == "beta" ? 1 : 0;
  }
  EXPECT_EQ(30u, from_alpha);
  EXPECT_EQ(30u, from_beta);
  EXPECT_EQ(2u, server.connections());
  // Same topic name on two robots: sequences are tracked per robot, so nothing looks lost.
  EXPECT_EQ(0u, server.totals().lost_messages);
}

TEST(EndToEnd, ServerClosesAConnectionThatDoesNotSpeakTheProtocol)
{
  ServerRunner server;
  ASSERT_TRUE(server.started()) << server.start_error();

  TcpTransportConfig config;
  config.host = "127.0.0.1";
  config.port = server.port();
  TcpTransport intruder(config);
  ASSERT_TRUE(intruder.connect());

  const std::string garbage =
    "GET / HTTP/1.1\r\nHost: localhost\r\nUser-Agent: not-a-robot\r\n\r\n";
  ASSERT_TRUE(
    intruder.send({{reinterpret_cast<const std::uint8_t *>(garbage.data()), garbage.size()}}));

  ASSERT_TRUE(wait_until([&server] {return !server.disconnect_reasons().empty();}));
  EXPECT_NE(std::string::npos, server.disconnect_reasons().front().find("protocol error"));
  EXPECT_TRUE(wait_until([&intruder] {return !intruder.is_connected();}));
  EXPECT_EQ(0u, server.data_count());
}

TEST(EndToEnd, ServerDropsARobotThatGoesSilent)
{
  ServerRunner server(0, milliseconds(200));
  ASSERT_TRUE(server.started()) << server.start_error();

  TcpTransportConfig config;
  config.host = "127.0.0.1";
  config.port = server.port();
  TcpTransport silent(config);
  ASSERT_TRUE(silent.connect());  // connected, but no HELLO, no data, no heartbeat

  ASSERT_TRUE(wait_until([&server] {return !server.disconnect_reasons().empty();}));
  EXPECT_NE(std::string::npos, server.disconnect_reasons().front().find("heartbeat"));
}

TEST(EndToEnd, HeartbeatsKeepAnIdleRobotConnected)
{
  ServerRunner server(0, milliseconds(400));
  ASSERT_TRUE(server.started()) << server.start_error();
  auto robot = make_robot(server.port(), "idle_robot");  // heartbeat every 100 ms

  ASSERT_TRUE(wait_until([&server] {return server.connections() == 1;}));
  std::this_thread::sleep_for(milliseconds(1200));
  // Three timeouts have passed without any sensor data, yet the robot was never dropped.
  EXPECT_TRUE(server.disconnect_reasons().empty());
  EXPECT_EQ(1u, robot->stats().connections);
  EXPECT_GE(server.totals().heartbeats, 5u);
}

}  // namespace robot_server_comm
