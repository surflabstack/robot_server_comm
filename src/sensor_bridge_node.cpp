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

#include "robot_server_comm/sensor_bridge_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "robot_server_comm/transport/transport_factory.hpp"

namespace robot_server_comm
{

namespace
{

using std::chrono::milliseconds;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::invalid_argument("invalid parameter: " + message);
  }
}

milliseconds seconds_to_ms(double seconds)
{
  return milliseconds(static_cast<std::int64_t>(seconds * 1000.0));
}

std::string absolute_topic(const std::string & topic)
{
  return (!topic.empty() && topic.front() != '/') ? "/" + topic : topic;
}

// Hidden topics have a name component starting with an underscore (ROS convention).
bool is_hidden(const std::string & topic)
{
  return topic.find("/_") != std::string::npos;
}

std::string join(const std::vector<std::string> & items, const std::string & separator)
{
  std::string text;
  for (const std::string & item : items) {
    text += (text.empty() ? "" : separator) + item;
  }
  return text;
}

}  // namespace

SensorBridgeNode::SensorBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("sensor_bridge", options),
  last_status_(std::chrono::steady_clock::now())
{
  parameters_ = load_parameters();

  TransportSettings transport_settings;
  transport_settings.host = parameters_.server_host;
  transport_settings.port = parameters_.server_port;
  transport_settings.connect_timeout = parameters_.connect_timeout;
  transport_settings.send_timeout = parameters_.send_timeout;

  const TransportFactory & factory = TransportFactory::instance();
  std::unique_ptr<Transport> transport = factory.create(parameters_.protocol, transport_settings);
  require(
    transport != nullptr,
    "protocol '" + parameters_.protocol + "' is not supported, available: " +
    join(factory.available_protocols(), ", "));

  TransmitterConfig transmitter_config;
  transmitter_config.robot_id = parameters_.robot_id;
  transmitter_config.max_queue_messages = parameters_.max_queue_messages;
  transmitter_config.max_queue_bytes = parameters_.max_queue_bytes;
  transmitter_config.reconnect_initial_delay = parameters_.reconnect_initial_delay;
  transmitter_config.reconnect_max_delay = parameters_.reconnect_max_delay;
  transmitter_config.heartbeat_period = parameters_.heartbeat_period;
  transmitter_config.enable_crc = parameters_.enable_crc;

  transmitter_ = std::make_unique<DataTransmitter>(
    std::move(transport), transmitter_config,
    [this](LogLevel level, const std::string & text) {forward_log(level, text);});

  if (parameters_.topics.empty()) {
    RCLCPP_INFO(
      get_logger(), "Forwarding every topic that appears to %s (robot id '%s')",
      transmitter_->endpoint().c_str(), parameters_.robot_id.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "Forwarding %s to %s (robot id '%s')",
      join(parameters_.topics, ", ").c_str(), transmitter_->endpoint().c_str(),
      parameters_.robot_id.c_str());
  }

  transmitter_->start();

  discovery_timer_ = create_wall_timer(parameters_.discovery_period, [this]() {discover_topics();});
  if (parameters_.status_period.count() > 0) {
    status_timer_ = create_wall_timer(parameters_.status_period, [this]() {report_status();});
  }
  discover_topics();
}

SensorBridgeNode::~SensorBridgeNode()
{
  // Stop the callbacks before the transmitter they use goes away.
  discovery_timer_.reset();
  status_timer_.reset();
  topics_.clear();
  if (transmitter_) {
    transmitter_->stop();
  }
}

template<typename ValueT>
ValueT SensorBridgeNode::declare(
  const std::string & name, const ValueT & default_value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  // Parameters are read once at start-up. Marking them read only makes a later
  // "ros2 param set" fail loudly instead of being silently ignored.
  descriptor.read_only = true;
  return declare_parameter<ValueT>(name, default_value, descriptor);
}

SensorBridgeNode::Parameters SensorBridgeNode::load_parameters()
{
  using StringList = std::vector<std::string>;
  Parameters parameters;

  parameters.protocol = declare<std::string>(
    "protocol", "tcp", "Transport used to reach the server.");
  parameters.server_host = declare<std::string>(
    "server_host", "127.0.0.1", "Host name or IP address of the Mission Planner Server.");
  const std::int64_t server_port = declare<std::int64_t>(
    "server_port", 9000, "Port of the Mission Planner Server.");
  parameters.robot_id = declare<std::string>(
    "robot_id", "robot_1", "Name announced to the server when connecting.");

  const StringList topics = declare<StringList>(
    "topics", StringList{},
    "Topics to forward. Leave empty to forward every topic that appears.");
  const StringList excluded = declare<StringList>(
    "exclude_topics",
    StringList{"/rosout", "/parameter_events", "/clock", "/events/read_split",
      "/events/write_split"},
    "Topics never forwarded when 'topics' is empty.");
  const StringList rate_limits = declare<StringList>(
    "topic_rate_limits", StringList{},
    "Per topic maximum forwarding rate, written 'topic:max_hz'.");

  const double discovery_period_s = declare<double>(
    "discovery_period_s", 0.5, "Period at which new topics are looked for, in seconds.");
  parameters.qos_reliability = declare<std::string>(
    "qos_reliability", "auto",
    "Subscription reliability: 'auto' (match the publishers), 'reliable' or 'best_effort'.");
  const std::int64_t qos_depth = declare<std::int64_t>(
    "qos_depth", 50, "History depth of the subscriptions.");

  const std::int64_t max_queue_messages = declare<std::int64_t>(
    "max_queue_messages", 200, "Messages kept while waiting for the link.");
  const std::int64_t max_queue_megabytes = declare<std::int64_t>(
    "max_queue_megabytes", 64, "Memory budget of the waiting messages, in megabytes.");

  const std::int64_t connect_timeout_ms = declare<std::int64_t>(
    "connect_timeout_ms", 2000, "Longest wait for a connection attempt, in milliseconds.");
  const std::int64_t send_timeout_ms = declare<std::int64_t>(
    "send_timeout_ms", 2000, "Longest time a send may stall, in milliseconds.");
  const std::int64_t reconnect_initial_delay_ms = declare<std::int64_t>(
    "reconnect_initial_delay_ms", 500, "First delay between reconnection attempts.");
  const std::int64_t reconnect_max_delay_ms = declare<std::int64_t>(
    "reconnect_max_delay_ms", 8000, "Largest delay between reconnection attempts.");
  const std::int64_t heartbeat_period_ms = declare<std::int64_t>(
    "heartbeat_period_ms", 1000, "Idle time after which a heartbeat frame is sent.");
  parameters.enable_crc = declare<bool>(
    "enable_crc", true, "Protect every frame body with a CRC-32.");

  const double status_period_s = declare<double>(
    "status_period_s", 5.0, "Period of the status report, 0 disables it.");
  const double log_period_s = declare<double>(
    "log_period_s", 2.0,
    "Shortest time between two 'Forwarding' log lines of one topic, 0 disables them.");

  // Validation: fail at start-up with a clear message rather than misbehave later.
  require(!parameters.server_host.empty(), "server_host is empty");
  require(
    is_valid_port(server_port),
    "server_port must be between 1 and 65535, got " + std::to_string(server_port));
  require(!parameters.robot_id.empty(), "robot_id is empty");
  require(parameters.robot_id.size() <= 128, "robot_id is longer than 128 characters");
  require(
    parameters.qos_reliability == "auto" || parameters.qos_reliability == "reliable" ||
    parameters.qos_reliability == "best_effort",
    "qos_reliability must be 'auto', 'reliable' or 'best_effort', got '" +
    parameters.qos_reliability + "'");
  require(qos_depth >= 1 && qos_depth <= 1000, "qos_depth must be between 1 and 1000");
  require(
    discovery_period_s >= 0.1 && discovery_period_s <= 60.0,
    "discovery_period_s must be between 0.1 and 60");
  require(
    max_queue_messages >= 1 && max_queue_messages <= 1000000,
    "max_queue_messages must be between 1 and 1000000");
  require(
    max_queue_megabytes >= 1 && max_queue_megabytes <= 4096,
    "max_queue_megabytes must be between 1 and 4096");
  require(
    connect_timeout_ms >= 100 && connect_timeout_ms <= 60000,
    "connect_timeout_ms must be between 100 and 60000");
  require(
    send_timeout_ms >= 100 && send_timeout_ms <= 60000,
    "send_timeout_ms must be between 100 and 60000");
  require(
    reconnect_initial_delay_ms >= 10 && reconnect_initial_delay_ms <= 60000,
    "reconnect_initial_delay_ms must be between 10 and 60000");
  require(
    reconnect_max_delay_ms >= reconnect_initial_delay_ms && reconnect_max_delay_ms <= 600000,
    "reconnect_max_delay_ms must be between reconnect_initial_delay_ms and 600000");
  require(
    heartbeat_period_ms >= 100 && heartbeat_period_ms <= 60000,
    "heartbeat_period_ms must be between 100 and 60000");
  require(
    status_period_s >= 0.0 && status_period_s <= 3600.0,
    "status_period_s must be between 0 and 3600");
  require(log_period_s >= 0.0 && log_period_s <= 3600.0, "log_period_s must be between 0 and 3600");

  parameters.server_port = static_cast<std::uint16_t>(server_port);
  for (const std::string & topic : clean_string_list(topics)) {
    parameters.topics.push_back(absolute_topic(topic));
  }
  for (const std::string & topic : clean_string_list(excluded)) {
    parameters.excluded_topics.insert(absolute_topic(topic));
  }
  for (const auto & limit : parse_rate_limits(rate_limits)) {
    parameters.rate_limits[absolute_topic(limit.first)] = limit.second;
  }
  parameters.discovery_period = seconds_to_ms(discovery_period_s);
  parameters.qos_depth = static_cast<std::size_t>(qos_depth);
  parameters.max_queue_messages = static_cast<std::size_t>(max_queue_messages);
  parameters.max_queue_bytes = static_cast<std::size_t>(max_queue_megabytes) * 1024u * 1024u;
  parameters.connect_timeout = milliseconds(connect_timeout_ms);
  parameters.send_timeout = milliseconds(send_timeout_ms);
  parameters.reconnect_initial_delay = milliseconds(reconnect_initial_delay_ms);
  parameters.reconnect_max_delay = milliseconds(reconnect_max_delay_ms);
  parameters.heartbeat_period = milliseconds(heartbeat_period_ms);
  parameters.status_period = seconds_to_ms(status_period_s);
  parameters.log_period = seconds_to_ms(log_period_s);
  return parameters;
}

void SensorBridgeNode::discover_topics()
{
  for (const auto & entry : get_topic_names_and_types()) {
    const std::string & topic = entry.first;
    const std::vector<std::string> & types = entry.second;

    if (topics_.count(topic) > 0 || skipped_topics_.count(topic) > 0 || !is_selected(topic)) {
      continue;
    }
    if (count_publishers(topic) == 0) {
      continue;  // Nobody publishes yet, look again at the next discovery.
    }
    if (types.size() != 1) {
      RCLCPP_WARN(
        get_logger(), "Topic '%s' is published with %zu different types, it is skipped",
        topic.c_str(), types.size());
      skipped_topics_.insert(topic);
      continue;
    }
    add_topic(topic, types.front());
  }
}

bool SensorBridgeNode::is_selected(const std::string & topic) const
{
  if (!parameters_.topics.empty()) {
    return std::find(parameters_.topics.begin(), parameters_.topics.end(), topic) !=
           parameters_.topics.end();
  }
  return !is_hidden(topic) && parameters_.excluded_topics.count(topic) == 0;
}

rclcpp::QoS SensorBridgeNode::make_qos(const std::string & topic) const
{
  rclcpp::QoS qos{rclcpp::KeepLast(parameters_.qos_depth)};
  if (parameters_.qos_reliability == "reliable") {
    return qos.reliable();
  }
  if (parameters_.qos_reliability == "best_effort") {
    return qos.best_effort();
  }

  // "auto": ask for the strongest guarantees every publisher of the topic can
  // offer. Asking for more than a publisher offers would silently prevent the
  // connection (incompatible QoS), asking for less would lose data for nothing.
  const auto publishers = get_publishers_info_by_topic(topic);
  bool all_reliable = !publishers.empty();
  bool all_transient_local = !publishers.empty();
  for (const rclcpp::TopicEndpointInfo & publisher : publishers) {
    const rmw_qos_profile_t & offered = publisher.qos_profile().get_rmw_qos_profile();
    all_reliable = all_reliable && offered.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    all_transient_local = all_transient_local &&
      offered.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }
  if (all_reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  if (all_transient_local) {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }
  return qos;
}

void SensorBridgeNode::add_topic(const std::string & topic, const std::string & type)
{
  auto bridged = std::make_unique<BridgedTopic>();
  bridged->name = topic;
  bridged->type = type;
  const auto limit = parameters_.rate_limits.find(topic);
  const double max_hz = limit != parameters_.rate_limits.end() ? limit->second : 0.0;
  bridged->limiter = RateLimiter(max_hz);

  const rclcpp::QoS qos = make_qos(topic);
  bridged->callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = bridged->callback_group;
  BridgedTopic * target = bridged.get();
  try {
    // A generic subscription delivers the message still serialized. The node
    // therefore needs no compile time knowledge of the message type, and a new
    // sensor topic is supported without changing or rebuilding this package.
    bridged->subscription = create_generic_subscription(
      topic, type, qos,
      [this, target](std::shared_ptr<const rclcpp::SerializedMessage> message) {
        on_message(*target, *message);
      },
      options);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot subscribe to '%s' [%s], the topic is skipped. Is the package that defines this "
      "message type installed? (%s)", topic.c_str(), type.c_str(), error.what());
    skipped_topics_.insert(topic);
    return;
  }

  const bool reliable =
    qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  if (max_hz > 0.0) {
    RCLCPP_INFO(
      get_logger(), "Subscribed to '%s' [%s] (%s, limited to %.1f Hz)", topic.c_str(),
      type.c_str(), reliable ? "reliable" : "best effort", max_hz);
  } else {
    RCLCPP_INFO(
      get_logger(), "Subscribed to '%s' [%s] (%s)", topic.c_str(), type.c_str(),
      reliable ? "reliable" : "best effort");
  }
  topics_.emplace(topic, std::move(bridged));
}

void SensorBridgeNode::on_message(BridgedTopic & topic, const rclcpp::SerializedMessage & message)
{
  ++topic.received;
  const auto now = std::chrono::steady_clock::now();
  if (!topic.limiter.allow(now)) {
    ++topic.throttled;
    return;
  }

  // Only a memory copy into the transmitter queue happens here. All network
  // work is done by the transmitter thread, so this callback stays short even
  // when the server is slow or unreachable.
  const rcl_serialized_message_t & raw = message.get_rcl_serialized_message();
  if (!transmitter_->submit(topic.name, topic.type, raw.buffer, raw.buffer_length)) {
    const std::uint64_t refused = ++topic.refused;
    if (refused == 1 || now - topic.last_refusal_log >= std::chrono::seconds(5)) {
      topic.last_refusal_log = now;
      RCLCPP_WARN(
        get_logger(),
        "A message of '%s' (%zu bytes) is larger than the queue budget and was refused "
        "(%" PRIu64 " refused so far)", topic.name.c_str(), raw.buffer_length, refused);
    }
    return;
  }
  const std::uint64_t forwarded = ++topic.forwarded;

  if (parameters_.log_period.count() > 0 && now - topic.last_log >= parameters_.log_period) {
    topic.last_log = now;
    RCLCPP_INFO(
      get_logger(), "%s '%s' [%s] %zu bytes (message %" PRIu64 " of this topic)",
      transmitter_->connected() ? "Forwarding" : "Link down, buffering",
      topic.name.c_str(), topic.type.c_str(), raw.buffer_length, forwarded);
  }
}

void SensorBridgeNode::report_status()
{
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_s = std::chrono::duration<double>(now - last_status_).count();
  last_status_ = now;

  const TransmitterStats stats = transmitter_->stats();
  std::ostringstream text;
  text << "Link " << (stats.connected ? "UP" : "DOWN") << " to " << transmitter_->endpoint() <<
    " | sent=" << stats.sent <<
    " queued=" << stats.queued_messages <<
    " dropped=" << stats.dropped_queue_full <<
    " reconnections=" << (stats.connections > 0 ? stats.connections - 1 : 0);
  if (topics_.empty()) {
    text << " | no topic yet, waiting for publishers";
  }
  for (auto & entry : topics_) {
    BridgedTopic & topic = *entry.second;
    const std::uint64_t forwarded = topic.forwarded.load();
    const double rate_hz = elapsed_s > 0.0 ?
      static_cast<double>(forwarded - topic.forwarded_at_last_status) / elapsed_s : 0.0;
    topic.forwarded_at_last_status = forwarded;
    text << " | " << topic.name << " " << std::fixed << std::setprecision(1) << rate_hz << " Hz";
  }
  RCLCPP_INFO(get_logger(), "%s", text.str().c_str());
}

void SensorBridgeNode::forward_log(LogLevel level, const std::string & text) const
{
  switch (level) {
    case LogLevel::kDebug:
      RCLCPP_DEBUG(get_logger(), "%s", text.c_str());
      break;
    case LogLevel::kInfo:
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
      break;
    case LogLevel::kWarn:
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
      break;
    case LogLevel::kError:
      RCLCPP_ERROR(get_logger(), "%s", text.c_str());
      break;
  }
}

}  // namespace robot_server_comm

// Makes the node loadable in a component container, next to other nodes of the robot.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_server_comm::SensorBridgeNode)
