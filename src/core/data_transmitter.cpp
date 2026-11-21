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

#include "robot_server_comm/core/data_transmitter.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "robot_server_comm/protocol/frame_codec.hpp"

namespace robot_server_comm
{

namespace
{

using std::chrono::milliseconds;

constexpr milliseconds kMinHeartbeatPeriod{100};
constexpr milliseconds kMinReconnectDelay{10};

std::uint64_t unix_time_ns()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

// Brings out of range settings back to safe values instead of failing later at run time.
TransmitterConfig sanitize(TransmitterConfig config)
{
  config.heartbeat_period = std::max(config.heartbeat_period, kMinHeartbeatPeriod);
  config.reconnect_initial_delay = std::max(config.reconnect_initial_delay, kMinReconnectDelay);
  config.reconnect_max_delay = std::max(config.reconnect_max_delay, config.reconnect_initial_delay);
  config.max_queue_messages = std::max<std::size_t>(config.max_queue_messages, 1);
  config.max_queue_bytes = std::max<std::size_t>(config.max_queue_bytes, 1);
  return config;
}

std::unique_ptr<Transport> require_transport(std::unique_ptr<Transport> transport)
{
  if (!transport) {
    throw std::invalid_argument("DataTransmitter needs a transport");
  }
  return transport;
}

}  // namespace

DataTransmitter::DataTransmitter(
  std::unique_ptr<Transport> transport, TransmitterConfig config, LogCallback logger)
: transport_(require_transport(std::move(transport))),
  config_(sanitize(std::move(config))),
  logger_(std::move(logger)),
  endpoint_(transport_->description()),
  queue_(config_.max_queue_messages, config_.max_queue_bytes)
{
}

DataTransmitter::~DataTransmitter()
{
  stop();
}

void DataTransmitter::start()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (started_) {
    return;
  }
  started_ = true;
  running_.store(true);
  worker_ = std::thread(&DataTransmitter::run, this);
}

void DataTransmitter::stop()
{
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    running_.store(false);
  }
  queue_.close();
  stop_condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool DataTransmitter::submit(
  const std::string & topic, const std::string & message_type,
  const std::uint8_t * data, std::size_t size)
{
  submitted_.fetch_add(1);

  const bool valid =
    !topic.empty() && topic.size() <= kMaxNameLength &&
    message_type.size() <= kMaxNameLength &&
    (data != nullptr || size == 0) &&
    size <= std::numeric_limits<std::uint32_t>::max();
  if (!valid) {
    rejected_.fetch_add(1);
    return false;
  }

  Frame frame;
  frame.type = FrameType::kData;
  frame.topic = topic;
  frame.message_type = message_type;
  frame.timestamp_ns = unix_time_ns();
  frame.sequence = next_sequence(topic);
  if (size > 0) {
    frame.payload.assign(data, data + size);
  }

  const auto result = queue_.push(std::move(frame), size);
  if (result.dropped_items > 0) {
    dropped_queue_full_.fetch_add(result.dropped_items);
  }
  if (!result.accepted) {
    rejected_.fetch_add(1);
  }
  return result.accepted;
}

TransmitterStats DataTransmitter::stats() const
{
  TransmitterStats stats;
  stats.connected = connected_.load();
  stats.submitted = submitted_.load();
  stats.sent = sent_.load();
  stats.dropped_queue_full = dropped_queue_full_.load();
  stats.rejected = rejected_.load();
  stats.send_failures = send_failures_.load();
  stats.connections = connections_.load();
  stats.bytes_sent = bytes_sent_.load();
  stats.queued_messages = queue_.size();
  stats.queued_bytes = queue_.bytes();
  return stats;
}

void DataTransmitter::run()
{
  milliseconds backoff = config_.reconnect_initial_delay;
  Clock::time_point next_attempt = Clock::now();
  Clock::time_point last_send = Clock::now();
  // Message taken from the queue and not delivered yet. It survives a
  // reconnection, so a short network glitch does not lose it.
  std::optional<Frame> in_flight;

  while (running_.load()) {
    if (!transport_->is_connected()) {
      if (connected_.exchange(false)) {
        log(
          LogLevel::kWarn,
          "link to " + endpoint_ + " lost (" + transport_->last_error() + "), reconnecting");
        backoff = config_.reconnect_initial_delay;
        next_attempt = Clock::now();
      }

      const Clock::time_point now = Clock::now();
      if (now < next_attempt) {
        const auto delay = std::chrono::duration_cast<milliseconds>(next_attempt - now);
        wait_or_stop(delay + milliseconds(1));
        continue;
      }

      Frame hello;
      hello.type = FrameType::kHello;
      hello.timestamp_ns = unix_time_ns();
      hello.payload.assign(config_.robot_id.begin(), config_.robot_id.end());

      if (transport_->connect() && send_frame(hello)) {
        connected_.store(true);
        connections_.fetch_add(1);
        backoff = config_.reconnect_initial_delay;
        last_send = Clock::now();
        log(LogLevel::kInfo, "connected to " + endpoint_ + " as '" + config_.robot_id + "'");
      } else {
        log(
          LogLevel::kWarn,
          "cannot reach " + endpoint_ + " (" + transport_->last_error() + "), next attempt in " +
          std::to_string(backoff.count()) + " ms");
        transport_->disconnect();
        next_attempt = Clock::now() + backoff;
        backoff = std::min(backoff * 2, config_.reconnect_max_delay);
      }
      continue;
    }

    if (!in_flight) {
      const auto idle = std::chrono::duration_cast<milliseconds>(Clock::now() - last_send);
      const milliseconds wait =
        idle >= config_.heartbeat_period ? milliseconds(0) : config_.heartbeat_period - idle;
      in_flight = queue_.pop(wait);
    }

    if (in_flight) {
      if (send_frame(*in_flight)) {
        sent_.fetch_add(1);
        last_send = Clock::now();
        in_flight.reset();
      } else {
        // The transport closed itself. The next iteration reconnects and retries this frame.
        send_failures_.fetch_add(1);
      }
      continue;
    }

    if (Clock::now() - last_send >= config_.heartbeat_period) {
      Frame heartbeat;
      heartbeat.type = FrameType::kHeartbeat;
      heartbeat.timestamp_ns = unix_time_ns();
      send_frame(heartbeat);
      last_send = Clock::now();
    }
  }

  transport_->disconnect();
  connected_.store(false);
}

bool DataTransmitter::send_frame(const Frame & frame)
{
  std::vector<std::uint8_t> header;
  try {
    header = encode_frame_header(frame, config_.enable_crc);
  } catch (const std::invalid_argument & error) {
    // submit() validates its input, so this only guards against future misuse.
    log(LogLevel::kError, std::string("frame dropped, it cannot be encoded: ") + error.what());
    return true;
  }

  const std::vector<ConstBuffer> buffers = {
    {header.data(), header.size()},
    {frame.payload.data(), frame.payload.size()},
  };
  if (!transport_->send(buffers)) {
    return false;
  }
  bytes_sent_.fetch_add(header.size() + frame.payload.size());
  return true;
}

bool DataTransmitter::wait_or_stop(std::chrono::milliseconds duration)
{
  std::unique_lock<std::mutex> lock(lifecycle_mutex_);
  return stop_condition_.wait_for(lock, duration, [this] {return !running_.load();});
}

std::uint64_t DataTransmitter::next_sequence(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(sequence_mutex_);
  return sequences_[topic]++;
}

void DataTransmitter::log(LogLevel level, const std::string & text) const
{
  if (logger_) {
    logger_(level, text);
    return;
  }
  if (level != LogLevel::kDebug) {
    std::cerr << "[transmitter] " << text << std::endl;
  }
}

}  // namespace robot_server_comm
