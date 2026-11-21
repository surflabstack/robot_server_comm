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

#ifndef ROBOT_SERVER_COMM__SERVER__PAYLOAD_SUMMARY_HPP_
#define ROBOT_SERVER_COMM__SERVER__PAYLOAD_SUMMARY_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace robot_server_comm
{

/// Turns the CDR payload of known message types into one readable line.
///
/// The server uses it to show WHAT it received (an acceleration, a GNSS
/// position, the size of a point cloud) and not only how many bytes arrived.
/// Unknown types fall back to a hex preview, so every topic is displayed.
class PayloadSummarizer
{
public:
  /// Decoder of one message type. It may throw CdrError on malformed input.
  using Decoder = std::function<std::string(const std::uint8_t * data, std::size_t size)>;

  /// Summarizer with the common sensor_msgs types already registered.
  PayloadSummarizer();

  /// Adds (or replaces) the decoder of a message type such as "sensor_msgs/msg/Imu".
  void register_decoder(const std::string & message_type, Decoder decoder);

  /// Returns true when a decoder exists for the type.
  bool knows(const std::string & message_type) const;

  /// One line describing the payload. Never throws.
  std::string summarize(const std::string & message_type, const std::vector<std::uint8_t> & payload)
  const;

private:
  std::map<std::string, Decoder> decoders_;
};

/// Formats a byte count with a unit, for example "1.3 MB".
std::string format_size(std::size_t bytes);

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__SERVER__PAYLOAD_SUMMARY_HPP_
