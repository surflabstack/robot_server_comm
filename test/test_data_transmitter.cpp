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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "robot_server_comm/core/data_transmitter.hpp"
#include "robot_server_comm/protocol/frame_codec.hpp"
#include "test_helpers.hpp"

namespace robot_server_comm
{

using std::chrono::milliseconds;
using test_support::wait_until;

namespace
{

/// What the fake transport saw, shared between the test and the transmitter thread.
struct FakeLink
{
  std::mutex mutex;
  bool accept_connections{true};
  bool connected{false};
  int sends_to_fail{0};
  int connect_calls{0};
  FrameDecoder decoder;
  std::vector<Frame> frames;

  std::vector<Frame> frames_of(FrameType type)
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<Frame> selected;
    for (const Frame & frame : frames) {
      if (frame.type == type) {
        selected.push_back(frame);
      }
    }
    return selected;
  }

  std::size_t count(FrameType type) {return frames_of(type).size();}

  void set_accept_connections(bool accept)
  {
    std::lock_guard<std::mutex> lock(mutex);
    accept_connections = accept;
  }

  void fail_next_sends(int count_to_fail)
  {
    std::lock_guard<std::mutex> lock(mutex);
    sends_to_fail = count_to_fail;
  }

  int connect_call_count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return connect_calls;
  }
};

/// In memory transport: no socket, and failures can be scripted by the test.
/// It also proves that the transmitter only depends on the Transport interface.
class FakeTransport : public Transport
{
public:
  explicit FakeTransport(std::shared_ptr<FakeLink> link)
  : link_(std::move(link)) {}

  bool connect() override
  {
    std::lock_guard<std::mutex> lock(link_->mutex);
    ++link_->connect_calls;
    if (!link_->accept_connections) {
      last_error_ = "connection refused (simulated)";
      return false;
    }
    link_->connected = true;
    link_->decoder.reset();
    return true;
  }

  void disconnect() override
  {
    std::lock_guard<std::mutex> lock(link_->mutex);
    link_->connected = false;
  }

  bool is_connected() override
  {
    std::lock_guard<std::mutex> lock(link_->mutex);
    return link_->connected;
  }

  bool send(const std::vector<ConstBuffer> & buffers) override
  {
    std::lock_guard<std::mutex> lock(link_->mutex);
    if (!link_->connected) {
      last_error_ = "not connected";
      return false;
    }
    if (link_->sends_to_fail > 0) {
      --link_->sends_to_fail;
      link_->connected = false;
      last_error_ = "send failed (simulated)";
      return false;
    }
    for (const ConstBuffer & buffer : buffers) {
      link_->decoder.feed(buffer.data, buffer.size);
    }
    Frame frame;
    while (link_->decoder.next(frame) == DecodeStatus::kFrameReady) {
      link_->frames.push_back(frame);
    }
    return true;
  }

  std::string last_error() const override {return last_error_;}
  std::string description() const override {return "fake://link";}

private:
  std::shared_ptr<FakeLink> link_;
  std::string last_error_;
};

TransmitterConfig fast_config()
{
  TransmitterConfig config;
  config.robot_id = "test_robot";
  config.reconnect_initial_delay = milliseconds(20);
  config.reconnect_max_delay = milliseconds(80);
  config.heartbeat_period = milliseconds(100);
  return config;
}

std::unique_ptr<DataTransmitter> make_transmitter(
  const std::shared_ptr<FakeLink> & link, const TransmitterConfig & config)
{
  // Logs are silenced: the scripted failures would only add noise to the test output.
  return std::make_unique<DataTransmitter>(
    std::make_unique<FakeTransport>(link), config, [](LogLevel, const std::string &) {});
}

bool submit_text(DataTransmitter & transmitter, const std::string & topic, const std::string & text)
{
  return transmitter.submit(
    topic, "std_msgs/msg/String", reinterpret_cast<const std::uint8_t *>(text.data()),
    text.size());
}

std::string text_of(const Frame & frame)
{
  return std::string(frame.payload.begin(), frame.payload.end());
}

}  // namespace

TEST(DataTransmitter, NeedsATransport)
{
  EXPECT_THROW(DataTransmitter(nullptr, TransmitterConfig()), std::invalid_argument);
}

TEST(DataTransmitter, AnnouncesTheRobotThenDeliversMessagesInOrder)
{
  auto link = std::make_shared<FakeLink>();
  auto transmitter = make_transmitter(link, fast_config());
  transmitter->start();

  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(submit_text(*transmitter, "/imu/data", "imu " + std::to_string(i)));
    EXPECT_TRUE(submit_text(*transmitter, "/gnss/fix", "gnss " + std::to_string(i)));
  }
  ASSERT_TRUE(wait_until([&link] {return link->count(FrameType::kData) == 20;}));

  {
    std::lock_guard<std::mutex> lock(link->mutex);
    ASSERT_FALSE(link->frames.empty());
    // The very first frame on a new connection identifies the robot.
    EXPECT_EQ(FrameType::kHello, link->frames.front().type);
    EXPECT_EQ("test_robot", text_of(link->frames.front()));
  }

  std::uint64_t next_imu = 0;
  std::uint64_t next_gnss = 0;
  for (const Frame & frame : link->frames_of(FrameType::kData)) {
    EXPECT_EQ("std_msgs/msg/String", frame.message_type);
    EXPECT_GT(frame.timestamp_ns, 0u);
    if (frame.topic == "/imu/data") {
      EXPECT_EQ(next_imu, frame.sequence);
      EXPECT_EQ("imu " + std::to_string(next_imu), text_of(frame));
      ++next_imu;
    } else {
      EXPECT_EQ("/gnss/fix", frame.topic);
      EXPECT_EQ(next_gnss, frame.sequence);
      EXPECT_EQ("gnss " + std::to_string(next_gnss), text_of(frame));
      ++next_gnss;
    }
  }
  EXPECT_EQ(10u, next_imu);
  EXPECT_EQ(10u, next_gnss);

  const TransmitterStats stats = transmitter->stats();
  EXPECT_TRUE(stats.connected);
  EXPECT_EQ(20u, stats.submitted);
  EXPECT_EQ(20u, stats.sent);
  EXPECT_EQ(0u, stats.dropped_queue_full);
  EXPECT_EQ(1u, stats.connections);
  EXPECT_GT(stats.bytes_sent, 20u * kFixedHeaderSize);
  EXPECT_EQ("fake://link", transmitter->endpoint());
}

TEST(DataTransmitter, ReconnectsAndRetriesTheMessageThatFailed)
{
  auto link = std::make_shared<FakeLink>();
  auto transmitter = make_transmitter(link, fast_config());
  transmitter->start();
  ASSERT_TRUE(wait_until([&transmitter] {return transmitter->stats().connected;}));

  link->fail_next_sends(1);
  EXPECT_TRUE(submit_text(*transmitter, "/imu/data", "first"));
  EXPECT_TRUE(submit_text(*transmitter, "/imu/data", "second"));
  ASSERT_TRUE(wait_until([&link] {return link->count(FrameType::kData) == 2;}));

  const std::vector<Frame> data = link->frames_of(FrameType::kData);
  // Nothing was lost and the order is preserved despite the broken connection.
  EXPECT_EQ("first", text_of(data[0]));
  EXPECT_EQ("second", text_of(data[1]));
  EXPECT_EQ(2u, link->count(FrameType::kHello));

  const TransmitterStats stats = transmitter->stats();
  EXPECT_EQ(2u, stats.connections);
  EXPECT_EQ(1u, stats.send_failures);
  EXPECT_EQ(2u, stats.sent);
}

TEST(DataTransmitter, KeepsOnlyTheNewestMessagesWhileTheServerIsUnreachable)
{
  auto link = std::make_shared<FakeLink>();
  link->set_accept_connections(false);
  TransmitterConfig config = fast_config();
  config.max_queue_messages = 5;
  auto transmitter = make_transmitter(link, config);
  transmitter->start();

  for (int i = 0; i < 20; ++i) {
    EXPECT_TRUE(submit_text(*transmitter, "/imu/data", std::to_string(i)));
  }
  TransmitterStats stats = transmitter->stats();
  EXPECT_FALSE(stats.connected);
  EXPECT_EQ(5u, stats.queued_messages);
  EXPECT_EQ(15u, stats.dropped_queue_full);
  EXPECT_EQ(0u, stats.sent);

  // The server comes back: the 5 most recent messages are delivered.
  link->set_accept_connections(true);
  ASSERT_TRUE(wait_until([&link] {return link->count(FrameType::kData) == 5;}));
  const std::vector<Frame> data = link->frames_of(FrameType::kData);
  for (std::size_t i = 0; i < data.size(); ++i) {
    EXPECT_EQ(15u + i, data[i].sequence);
    EXPECT_EQ(std::to_string(15 + i), text_of(data[i]));
  }
}

TEST(DataTransmitter, RespectsTheByteBudgetOfTheQueue)
{
  auto link = std::make_shared<FakeLink>();
  link->set_accept_connections(false);
  TransmitterConfig config = fast_config();
  config.max_queue_messages = 1000;
  config.max_queue_bytes = 1000;
  auto transmitter = make_transmitter(link, config);
  transmitter->start();

  const std::string block(400, 'x');
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(submit_text(*transmitter, "/camera/image", block));
  }
  const TransmitterStats stats = transmitter->stats();
  EXPECT_EQ(2u, stats.queued_messages);
  EXPECT_EQ(800u, stats.queued_bytes);
  EXPECT_EQ(3u, stats.dropped_queue_full);
}

TEST(DataTransmitter, WaitsLongerAndLongerBetweenConnectionAttempts)
{
  auto link = std::make_shared<FakeLink>();
  link->set_accept_connections(false);
  TransmitterConfig config = fast_config();
  config.reconnect_initial_delay = milliseconds(50);
  config.reconnect_max_delay = milliseconds(200);
  auto transmitter = make_transmitter(link, config);
  transmitter->start();

  std::this_thread::sleep_for(milliseconds(1000));
  // Attempts at about 0, 50, 150, 350, 550, 750 and 950 ms: 7 with exponential
  // backoff capped at 200 ms. A fixed 50 ms retry would have made about 20.
  const int attempts = link->connect_call_count();
  EXPECT_GE(attempts, 4);
  EXPECT_LE(attempts, 9);
}

TEST(DataTransmitter, SendsHeartbeatsWhenThereIsNothingElseToSend)
{
  auto link = std::make_shared<FakeLink>();
  auto transmitter = make_transmitter(link, fast_config());
  transmitter->start();
  EXPECT_TRUE(wait_until([&link] {return link->count(FrameType::kHeartbeat) >= 3;}));
  EXPECT_EQ(0u, link->count(FrameType::kData));
}

TEST(DataTransmitter, RefusesInvalidMessages)
{
  auto link = std::make_shared<FakeLink>();
  TransmitterConfig config = fast_config();
  config.max_queue_bytes = 100;
  auto transmitter = make_transmitter(link, config);

  const std::string small = "ok";
  const std::string too_big(101, 'x');
  EXPECT_FALSE(submit_text(*transmitter, "", small));
  EXPECT_FALSE(submit_text(*transmitter, std::string(kMaxNameLength + 1, 't'), small));
  EXPECT_FALSE(submit_text(*transmitter, "/imu/data", too_big));
  EXPECT_FALSE(transmitter->submit("/imu/data", "std_msgs/msg/String", nullptr, 10));
  EXPECT_TRUE(submit_text(*transmitter, "/imu/data", small));

  const TransmitterStats stats = transmitter->stats();
  EXPECT_EQ(5u, stats.submitted);
  EXPECT_EQ(4u, stats.rejected);
  EXPECT_EQ(1u, stats.queued_messages);
}

TEST(DataTransmitter, SubmitNeverWaitsForTheNetwork)
{
  auto link = std::make_shared<FakeLink>();
  link->set_accept_connections(false);
  auto transmitter = make_transmitter(link, fast_config());
  transmitter->start();

  const std::string scan(100000, 'p');
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 1000; ++i) {
    submit_text(*transmitter, "/velodyne_points", scan);
  }
  // 1000 point cloud sized messages are queued (or dropped) while the server is down.
  EXPECT_LT(std::chrono::steady_clock::now() - start, milliseconds(2000));
}

TEST(DataTransmitter, StopsQuicklyEvenWhenTheServerIsUnreachable)
{
  auto link = std::make_shared<FakeLink>();
  link->set_accept_connections(false);
  TransmitterConfig config = fast_config();
  config.reconnect_initial_delay = milliseconds(5000);
  config.reconnect_max_delay = milliseconds(5000);
  auto transmitter = make_transmitter(link, config);
  transmitter->start();
  ASSERT_TRUE(wait_until([&link] {return link->connect_call_count() >= 1;}));

  const auto start = std::chrono::steady_clock::now();
  transmitter->stop();
  EXPECT_LT(std::chrono::steady_clock::now() - start, milliseconds(1000));
  EXPECT_FALSE(transmitter->stats().connected);
  // After stop() nothing is accepted any more.
  EXPECT_FALSE(submit_text(*transmitter, "/imu/data", "late"));
}

}  // namespace robot_server_comm
