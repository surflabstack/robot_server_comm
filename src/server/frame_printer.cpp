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

#include "robot_server_comm/server/frame_printer.hpp"

#include <iomanip>
#include <sstream>
#include <string>

namespace robot_server_comm
{

namespace
{

std::string robot_name(const ClientInfo & client)
{
  return client.robot_id.empty() ? "client#" + std::to_string(client.id) : client.robot_id;
}

// Time between the robot stamping the message and the server receiving it.
// It is only meaningful when both clocks are synchronized (always true on one machine).
double latency_ms(std::uint64_t sent_ns)
{
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  return (static_cast<double>(now_ns) - static_cast<double>(sent_ns)) / 1e6;
}

std::string fixed(double value, int decimals)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(decimals) << value;
  return text.str();
}

}  // namespace

FramePrinter::FramePrinter(std::ostream & output, FramePrinterConfig config)
: output_(output), config_(config), window_start_(Clock::now())
{
}

void FramePrinter::on_connect(const ClientInfo & client)
{
  output_ << "[SERVER] client #" << client.id << " connected from " << client.address << std::endl;
}

void FramePrinter::on_disconnect(const ClientInfo & client, const std::string & reason)
{
  output_ << "[SERVER] " << robot_name(client) << " (client #" << client.id << ", " <<
    client.address << ") disconnected: " << reason << std::endl;
}

void FramePrinter::on_frame(const ClientInfo & client, const Frame & frame)
{
  switch (frame.type) {
    case FrameType::kHello:
      output_ << "[HELLO] client #" << client.id << " is robot '" << client.robot_id <<
        "' (protocol v" << static_cast<int>(kProtocolVersion) << ")" << std::endl;
      break;
    case FrameType::kHeartbeat:
      ++totals_.heartbeats;
      break;
    case FrameType::kData:
      handle_data(client, frame);
      break;
  }
}

void FramePrinter::handle_data(const ClientInfo & client, const Frame & frame)
{
  const std::string robot = robot_name(client);
  if (topics_.empty()) {
    // Rates are measured from the first message, not from the start of the server.
    window_start_ = Clock::now();
  }
  TopicStats & stats = topics_[{robot, frame.topic}];
  stats.message_type = frame.message_type;

  // Sequence numbers are per topic and survive reconnections. A jump forward
  // means the robot dropped messages. A jump backward means the robot process
  // restarted, so counting simply starts again.
  std::uint64_t lost_now = 0;
  if (stats.has_sequence && frame.sequence > stats.next_sequence) {
    lost_now = frame.sequence - stats.next_sequence;
  }
  stats.has_sequence = true;
  stats.next_sequence = frame.sequence + 1;
  stats.lost += lost_now;
  totals_.lost_messages += lost_now;

  ++stats.received;
  ++stats.window_received;
  stats.bytes += frame.payload.size();
  stats.window_bytes += frame.payload.size();
  ++totals_.data_frames;
  totals_.data_bytes += frame.payload.size();

  const Clock::time_point now = Clock::now();
  const bool due = !stats.printed_once || config_.print_period.count() == 0 ||
    now - stats.last_print >= config_.print_period;
  if (!due && lost_now == 0) {
    return;
  }
  stats.printed_once = true;
  stats.last_print = now;

  output_ << "[DATA] robot=" << robot <<
    " topic=" << frame.topic <<
    " type=" << frame.message_type <<
    " seq=" << frame.sequence <<
    " size=" << format_size(frame.payload.size()) <<
    " latency=" << fixed(latency_ms(frame.timestamp_ns), 2) << " ms";
  if (lost_now > 0) {
    output_ << " LOST_BEFORE=" << lost_now;
  }
  output_ << " | " << summarizer_.summarize(frame.message_type, frame.payload) << std::endl;
}

void FramePrinter::tick()
{
  if (config_.stats_period.count() == 0) {
    return;
  }
  if (Clock::now() - window_start_ >= config_.stats_period) {
    print_stats();
  }
}

void FramePrinter::print_stats()
{
  const Clock::time_point now = Clock::now();
  const double window_s = std::chrono::duration<double>(now - window_start_).count();
  window_start_ = now;
  if (topics_.empty() || window_s <= 0.0) {
    return;
  }

  output_ << "[STATS] ---- last " << fixed(window_s, 1) << " s ---- total messages=" <<
    totals_.data_frames << " total data=" << format_size(totals_.data_bytes) <<
    " lost=" << totals_.lost_messages << std::endl;
  for (auto & entry : topics_) {
    TopicStats & stats = entry.second;
    const double rate_hz = static_cast<double>(stats.window_received) / window_s;
    const double bytes_per_s = static_cast<double>(stats.window_bytes) / window_s;
    std::ostringstream line;
    line << "[STATS] " << std::left << std::setw(14) << entry.first.first << ' ' <<
      std::setw(28) << entry.first.second << ' ' <<
      std::right << std::setw(7) << fixed(rate_hz, 1) << " Hz " <<
      std::setw(14) << (format_size(static_cast<std::size_t>(bytes_per_s)) + "/s") <<
      "  received=" << stats.received << " lost=" << stats.lost;
    output_ << line.str() << std::endl;
    stats.window_received = 0;
    stats.window_bytes = 0;
  }
}

}  // namespace robot_server_comm
