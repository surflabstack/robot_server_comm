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

#ifndef ROBOT_SERVER_COMM__SENSOR_BRIDGE_NODE_HPP_
#define ROBOT_SERVER_COMM__SENSOR_BRIDGE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "robot_server_comm/core/config_utils.hpp"
#include "robot_server_comm/core/data_transmitter.hpp"
#include "robot_server_comm/core/logging.hpp"

namespace robot_server_comm
{

/// ROS 2 node that subscribes to sensor topics and hands every message to a DataTransmitter.
///
/// The node knows nothing about sockets or wire formats, and the transmitter
/// knows nothing about ROS. The node only does the ROS side of the job:
///   - find the topics to forward (explicit list, or every topic that appears),
///   - subscribe to them whatever their message type (generic subscriptions,
///     so no code change is needed for a new sensor),
///   - pass the serialized message, its topic and its type to the transmitter,
///   - report what is going on in the terminal.
class SensorBridgeNode : public rclcpp::Node
{
public:
  explicit SensorBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SensorBridgeNode() override;

private:
  /// Validated copy of the node parameters.
  struct Parameters
  {
    std::string protocol;
    std::string server_host;
    std::uint16_t server_port{0};
    std::string robot_id;
    std::vector<std::string> topics;
    std::set<std::string> excluded_topics;
    std::map<std::string, double> rate_limits;
    std::chrono::milliseconds discovery_period{500};
    std::string qos_reliability;
    std::size_t qos_depth{50};
    std::size_t max_queue_messages{200};
    std::size_t max_queue_bytes{0};
    std::chrono::milliseconds connect_timeout{2000};
    std::chrono::milliseconds send_timeout{2000};
    std::chrono::milliseconds reconnect_initial_delay{500};
    std::chrono::milliseconds reconnect_max_delay{8000};
    std::chrono::milliseconds heartbeat_period{1000};
    bool enable_crc{true};
    std::chrono::milliseconds status_period{5000};
    std::chrono::milliseconds log_period{2000};
  };

  /// One forwarded topic with its subscription and counters.
  ///
  /// Each topic has its own callback group: with a multi-threaded executor the
  /// topics are handled in parallel (a 7 MB image never delays the 100 Hz IMU)
  /// while the messages of one topic stay in order. The counters are atomic
  /// because the status timer reads them from another thread.
  struct BridgedTopic
  {
    std::string name;
    std::string type;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::GenericSubscription::SharedPtr subscription;
    RateLimiter limiter;
    std::atomic<std::uint64_t> received{0};
    std::atomic<std::uint64_t> forwarded{0};
    std::atomic<std::uint64_t> throttled{0};
    std::atomic<std::uint64_t> refused{0};
    std::uint64_t forwarded_at_last_status{0};
    std::chrono::steady_clock::time_point last_log{};
    std::chrono::steady_clock::time_point last_refusal_log{};
  };

  template<typename ValueT>
  ValueT declare(
    const std::string & name, const ValueT & default_value, const std::string & description);

  Parameters load_parameters();
  void discover_topics();
  bool is_selected(const std::string & topic) const;
  rclcpp::QoS make_qos(const std::string & topic) const;
  void add_topic(const std::string & topic, const std::string & type);
  void on_message(BridgedTopic & topic, const rclcpp::SerializedMessage & message);
  void report_status();
  void forward_log(LogLevel level, const std::string & text) const;

  Parameters parameters_;
  std::unique_ptr<DataTransmitter> transmitter_;
  std::map<std::string, std::unique_ptr<BridgedTopic>> topics_;
  /// Topics that could not be subscribed to (reported once, then left alone).
  std::set<std::string> skipped_topics_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::chrono::steady_clock::time_point last_status_;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__SENSOR_BRIDGE_NODE_HPP_
