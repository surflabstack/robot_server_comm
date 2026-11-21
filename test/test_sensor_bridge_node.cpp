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

// Tests of the ROS 2 node itself: real publishers, the real node and a real
// server on the loopback interface, all in this process.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_server_comm/sensor_bridge_node.hpp"
#include "robot_server_comm/server/frame_server.hpp"

namespace robot_server_comm
{
namespace
{

using std::chrono::milliseconds;

/// Mission Planner Server running in a background thread, keeping every frame it receives.
class RecordingServer
{
public:
  RecordingServer()
  {
    FrameServerConfig config;
    config.port = 0;  // Let the system pick a free port.
    FrameServerHandlers handlers;
    handlers.on_frame = [this](const ClientInfo &, const Frame & frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.push_back(frame);
      };
    server_ = std::make_unique<FrameServer>(config, handlers);
    std::string error;
    if (!server_->start(error)) {
      throw std::runtime_error("cannot start the test server: " + error);
    }
    port_ = server_->port();
    thread_ = std::thread(
      [this]() {
        while (running_.load()) {
          server_->poll_once(milliseconds(20));
        }
      });
  }

  ~RecordingServer()
  {
    running_.store(false);
    thread_.join();
  }

  std::uint16_t port() const {return port_;}

  /// DATA frames received so far on one topic, in order of arrival.
  std::vector<Frame> data_frames(const std::string & topic) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Frame> selected;
    for (const Frame & frame : frames_) {
      if (frame.type == FrameType::kData && frame.topic == topic) {
        selected.push_back(frame);
      }
    }
    return selected;
  }

  /// Robot id announced in the first HELLO frame, empty when none arrived yet.
  std::string robot_id() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const Frame & frame : frames_) {
      if (frame.type == FrameType::kHello) {
        return std::string(frame.payload.begin(), frame.payload.end());
      }
    }
    return "";
  }

private:
  std::unique_ptr<FrameServer> server_;
  std::uint16_t port_{0};
  std::atomic<bool> running_{true};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::vector<Frame> frames_;
};

/// Turns the payload of a frame back into a typed ROS message.
template<typename MessageT>
MessageT deserialize(const Frame & frame)
{
  rclcpp::SerializedMessage serialized(frame.payload.size());
  auto & raw = serialized.get_rcl_serialized_message();
  std::copy(frame.payload.begin(), frame.payload.end(), raw.buffer);
  raw.buffer_length = frame.payload.size();

  MessageT message;
  rclcpp::Serialization<MessageT>().deserialize_message(&serialized, &message);
  return message;
}

class SensorBridgeNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  /// Node options for a bridge connected to `server`, plus test specific parameters.
  static rclcpp::NodeOptions bridge_options(
    const RecordingServer & server, std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("server_port", static_cast<std::int64_t>(server.port())),
      rclcpp::Parameter("robot_id", "test_robot"),
      rclcpp::Parameter("discovery_period_s", 0.1),
      rclcpp::Parameter("status_period_s", 0.0),
      rclcpp::Parameter("log_period_s", 0.0),
    };
    parameters.insert(parameters.end(), extra.begin(), extra.end());
    return rclcpp::NodeOptions().parameter_overrides(parameters);
  }

  /// Spins the nodes and calls `publish` every 20 ms until `done` is true or 10 s passed.
  static bool spin_until(
    rclcpp::executors::SingleThreadedExecutor & executor,
    const std::function<void()> & publish, const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      if (done()) {
        return true;
      }
      publish();
      executor.spin_some(milliseconds(20));
      std::this_thread::sleep_for(milliseconds(20));
    }
    return done();
  }
};

TEST_F(SensorBridgeNodeTest, forwards_topics_of_different_types_without_knowing_them)
{
  RecordingServer server;
  auto bridge = std::make_shared<SensorBridgeNode>(bridge_options(server));
  auto sensors = std::make_shared<rclcpp::Node>("test_sensors_all");
  auto text_publisher = sensors->create_publisher<std_msgs::msg::String>("/bridge_test/text", 10);
  auto imu_publisher = sensors->create_publisher<sensor_msgs::msg::Imu>("/bridge_test/imu", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(sensors);

  const bool received = spin_until(
    executor,
    [&]() {
      std_msgs::msg::String text;
      text.data = "hello mission planner";
      text_publisher->publish(text);

      sensor_msgs::msg::Imu imu;
      imu.header.frame_id = "imu_link";
      imu.linear_acceleration.z = 9.81;
      imu_publisher->publish(imu);
    },
    [&]() {
      return server.data_frames("/bridge_test/text").size() >= 3 &&
             server.data_frames("/bridge_test/imu").size() >= 3;
    });
  ASSERT_TRUE(received) << "the server did not receive both topics";

  EXPECT_EQ(server.robot_id(), "test_robot");

  const std::vector<Frame> text_frames = server.data_frames("/bridge_test/text");
  for (std::size_t i = 0; i < text_frames.size(); ++i) {
    EXPECT_EQ(text_frames[i].message_type, "std_msgs/msg/String");
    EXPECT_EQ(text_frames[i].sequence, i) << "sequence numbers must count from 0 without gaps";
    EXPECT_EQ(deserialize<std_msgs::msg::String>(text_frames[i]).data, "hello mission planner");
  }

  const std::vector<Frame> imu_frames = server.data_frames("/bridge_test/imu");
  for (const Frame & frame : imu_frames) {
    EXPECT_EQ(frame.message_type, "sensor_msgs/msg/Imu");
    const auto imu = deserialize<sensor_msgs::msg::Imu>(frame);
    EXPECT_EQ(imu.header.frame_id, "imu_link");
    EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 9.81);
  }
}

TEST_F(SensorBridgeNodeTest, forwards_only_the_listed_topics)
{
  RecordingServer server;
  auto bridge = std::make_shared<SensorBridgeNode>(
    bridge_options(
      server,
      {rclcpp::Parameter("topics", std::vector<std::string>{"/bridge_test/wanted"})}));
  auto sensors = std::make_shared<rclcpp::Node>("test_sensors_listed");
  auto wanted = sensors->create_publisher<std_msgs::msg::String>("/bridge_test/wanted", 10);
  auto unwanted = sensors->create_publisher<std_msgs::msg::String>("/bridge_test/unwanted", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(sensors);

  const bool received = spin_until(
    executor,
    [&]() {
      std_msgs::msg::String text;
      text.data = "data";
      wanted->publish(text);
      unwanted->publish(text);
    },
    [&]() {return server.data_frames("/bridge_test/wanted").size() >= 5;});
  ASSERT_TRUE(received) << "the listed topic did not reach the server";

  EXPECT_TRUE(server.data_frames("/bridge_test/unwanted").empty());
}

TEST_F(SensorBridgeNodeTest, refuses_to_start_with_invalid_parameters)
{
  RecordingServer server;
  const auto start_with = [&server](const rclcpp::Parameter & parameter) {
      SensorBridgeNode node(bridge_options(server, {parameter}));
    };

  EXPECT_THROW(
    start_with(rclcpp::Parameter("server_port", static_cast<std::int64_t>(70000))),
    std::invalid_argument);
  EXPECT_THROW(start_with(rclcpp::Parameter("server_host", "")), std::invalid_argument);
  EXPECT_THROW(start_with(rclcpp::Parameter("protocol", "carrier_pigeon")), std::invalid_argument);
  EXPECT_THROW(start_with(rclcpp::Parameter("qos_reliability", "maybe")), std::invalid_argument);
  EXPECT_THROW(
    start_with(rclcpp::Parameter("topic_rate_limits", std::vector<std::string>{"/imu:fast"})),
    std::invalid_argument);
}

}  // namespace
}  // namespace robot_server_comm
