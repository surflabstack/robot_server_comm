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

#ifndef ROBOT_SERVER_COMM__CORE__DATA_TRANSMITTER_HPP_
#define ROBOT_SERVER_COMM__CORE__DATA_TRANSMITTER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "robot_server_comm/core/bounded_queue.hpp"
#include "robot_server_comm/core/logging.hpp"
#include "robot_server_comm/protocol/frame.hpp"
#include "robot_server_comm/transport/transport.hpp"

namespace robot_server_comm
{

/// Settings of the transmitter.
struct TransmitterConfig
{
  /// Name announced to the server in the HELLO frame.
  std::string robot_id{"robot"};
  /// Largest number of messages waiting to be sent.
  std::size_t max_queue_messages{200};
  /// Largest total size of the messages waiting to be sent.
  std::size_t max_queue_bytes{64u * 1024u * 1024u};
  /// Delay before the first reconnection attempt. It doubles after each failure.
  std::chrono::milliseconds reconnect_initial_delay{500};
  /// Upper bound of the reconnection delay.
  std::chrono::milliseconds reconnect_max_delay{8000};
  /// A HEARTBEAT frame is sent when nothing else was sent for this long.
  std::chrono::milliseconds heartbeat_period{1000};
  /// Protect topic, type and payload of each frame with a CRC-32.
  bool enable_crc{true};
};

/// Counters describing what the transmitter did so far. Safe to read at any time.
struct TransmitterStats
{
  bool connected{false};
  std::uint64_t submitted{0};           ///< Messages handed to submit().
  std::uint64_t sent{0};                ///< DATA frames fully written to the link.
  std::uint64_t dropped_queue_full{0};  ///< Old messages discarded because the queue was full.
  std::uint64_t rejected{0};            ///< Messages refused (invalid or larger than the queue).
  std::uint64_t send_failures{0};       ///< Sends that failed and triggered a reconnection.
  std::uint64_t connections{0};         ///< Successful connections (1 + number of reconnections).
  std::uint64_t bytes_sent{0};          ///< Bytes written to the link, headers included.
  std::size_t queued_messages{0};
  std::size_t queued_bytes{0};
};

/// Sends sensor messages to the server, independently from ROS.
///
/// Responsibilities:
///   - take messages from any thread without blocking it (submit),
///   - buffer them in a bounded queue that drops the oldest data when full,
///   - own the transport: connect, detect failures, reconnect with exponential
///     backoff, announce the robot (HELLO) and keep the link alive (HEARTBEAT),
///   - frame each message with the wire protocol and write it to the transport.
///
/// One worker thread does all the network I/O, so a slow or dead link can never
/// stall the ROS executor that calls submit().
class DataTransmitter
{
public:
  DataTransmitter(
    std::unique_ptr<Transport> transport, TransmitterConfig config, LogCallback logger = nullptr);
  ~DataTransmitter();

  DataTransmitter(const DataTransmitter &) = delete;
  DataTransmitter & operator=(const DataTransmitter &) = delete;

  /// Starts the worker thread. Calling it twice has no effect.
  void start();

  /// Stops the worker thread and closes the link. Messages still queued are discarded.
  void stop();

  /// Queues one serialized message for transmission. Thread safe, never blocks on the network.
  ///
  /// \param topic ROS topic the message came from
  /// \param message_type ROS type name, for example "sensor_msgs/msg/Imu"
  /// \param data serialized message (copied)
  /// \param size number of bytes in `data`
  /// \return true when the message was queued, false when it was refused
  bool submit(
    const std::string & topic, const std::string & message_type,
    const std::uint8_t * data, std::size_t size);

  /// Snapshot of the counters.
  TransmitterStats stats() const;

  /// True while the link to the server is up. Cheap, safe to call from any thread.
  bool connected() const {return connected_.load();}

  /// Text describing the server end of the link.
  std::string endpoint() const {return endpoint_;}

private:
  using Clock = std::chrono::steady_clock;

  void run();
  bool send_frame(const Frame & frame);
  bool wait_or_stop(std::chrono::milliseconds duration);
  std::uint64_t next_sequence(const std::string & topic);
  void log(LogLevel level, const std::string & text) const;

  std::unique_ptr<Transport> transport_;
  const TransmitterConfig config_;
  const LogCallback logger_;
  const std::string endpoint_;

  BoundedQueue<Frame> queue_;

  std::mutex sequence_mutex_;
  std::unordered_map<std::string, std::uint64_t> sequences_;

  std::mutex lifecycle_mutex_;
  std::condition_variable stop_condition_;
  std::thread worker_;
  bool started_{false};
  std::atomic<bool> running_{false};

  std::atomic<bool> connected_{false};
  std::atomic<std::uint64_t> submitted_{0};
  std::atomic<std::uint64_t> sent_{0};
  std::atomic<std::uint64_t> dropped_queue_full_{0};
  std::atomic<std::uint64_t> rejected_{0};
  std::atomic<std::uint64_t> send_failures_{0};
  std::atomic<std::uint64_t> connections_{0};
  std::atomic<std::uint64_t> bytes_sent_{0};
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__CORE__DATA_TRANSMITTER_HPP_
