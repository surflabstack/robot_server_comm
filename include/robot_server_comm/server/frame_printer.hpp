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

#ifndef ROBOT_SERVER_COMM__SERVER__FRAME_PRINTER_HPP_
#define ROBOT_SERVER_COMM__SERVER__FRAME_PRINTER_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <ostream>
#include <string>
#include <utility>

#include "robot_server_comm/protocol/frame.hpp"
#include "robot_server_comm/server/frame_server.hpp"
#include "robot_server_comm/server/payload_summary.hpp"

namespace robot_server_comm
{

/// Settings of the terminal output of the server.
struct FramePrinterConfig
{
  /// Shortest time between two printed messages of the same topic.
  /// 0 prints every message (a 100 Hz IMU then fills the terminal quickly).
  std::chrono::milliseconds print_period{500};
  /// Period of the statistics table. 0 disables it.
  std::chrono::milliseconds stats_period{5000};
};

/// Totals kept by the printer, exposed for tests and for the final report.
struct ReceptionTotals
{
  std::uint64_t data_frames{0};
  std::uint64_t data_bytes{0};
  std::uint64_t lost_messages{0};
  std::uint64_t heartbeats{0};
};

/// Displays what the server receives and checks the stream for missing messages.
///
/// Every topic of every robot is tracked separately: message count, data rate,
/// and gaps in the sequence numbers (a gap means the robot dropped messages
/// because its queue was full, for example during a network outage).
class FramePrinter
{
public:
  FramePrinter(std::ostream & output, FramePrinterConfig config);

  void on_connect(const ClientInfo & client);
  void on_frame(const ClientInfo & client, const Frame & frame);
  void on_disconnect(const ClientInfo & client, const std::string & reason);

  /// Prints the statistics table when it is due. Call it regularly.
  void tick();

  /// Prints the statistics table now.
  void print_stats();

  /// Totals since the server started.
  const ReceptionTotals & totals() const {return totals_;}

  /// Gives access to the summarizer, for example to register more message types.
  PayloadSummarizer & summarizer() {return summarizer_;}

private:
  using Clock = std::chrono::steady_clock;

  struct TopicStats
  {
    std::string message_type;
    std::uint64_t received{0};
    std::uint64_t bytes{0};
    std::uint64_t lost{0};
    std::uint64_t window_received{0};
    std::uint64_t window_bytes{0};
    std::uint64_t next_sequence{0};
    bool has_sequence{false};
    bool printed_once{false};
    Clock::time_point last_print{};
  };

  void handle_data(const ClientInfo & client, const Frame & frame);

  std::ostream & output_;
  FramePrinterConfig config_;
  PayloadSummarizer summarizer_;
  ReceptionTotals totals_;
  /// Key: (robot id, topic).
  std::map<std::pair<std::string, std::string>, TopicStats> topics_;
  Clock::time_point window_start_;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__SERVER__FRAME_PRINTER_HPP_
