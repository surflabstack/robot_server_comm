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

#ifndef ROBOT_SERVER_COMM__PROTOCOL__FRAME_HPP_
#define ROBOT_SERVER_COMM__PROTOCOL__FRAME_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_server_comm
{

/// Version of the wire protocol implemented by this package.
constexpr std::uint8_t kProtocolVersion = 1;

/// Magic bytes that open every frame ("RSCP": Robot Server Communication Protocol).
constexpr std::uint8_t kFrameMagic[4] = {'R', 'S', 'C', 'P'};

/// Size in bytes of the fixed part of the frame header.
constexpr std::size_t kFixedHeaderSize = 40;

/// Longest topic name or message type name accepted in a frame.
constexpr std::size_t kMaxNameLength = 1024;

/// Default upper bound for a payload (protects the receiver from bogus lengths).
constexpr std::size_t kDefaultMaxPayloadSize = 64u * 1024u * 1024u;

/// Flag bit: the body CRC field of the header is filled in and must be checked.
constexpr std::uint16_t kFlagBodyCrc = 0x0001;

/// Kind of frame travelling on the link.
enum class FrameType : std::uint8_t
{
  kData = 1,       ///< One serialized sensor message.
  kHello = 2,      ///< First frame after connecting, the payload is the robot id.
  kHeartbeat = 3,  ///< Sent when the link is idle so the server can detect a dead robot.
};

/// Returns true when the raw value read from the wire is a known frame type.
inline bool is_valid_frame_type(std::uint8_t raw)
{
  return raw >= static_cast<std::uint8_t>(FrameType::kData) &&
         raw <= static_cast<std::uint8_t>(FrameType::kHeartbeat);
}

/// Human readable name of a frame type (for logs).
inline const char * to_string(FrameType type)
{
  switch (type) {
    case FrameType::kData: return "DATA";
    case FrameType::kHello: return "HELLO";
    case FrameType::kHeartbeat: return "HEARTBEAT";
  }
  return "UNKNOWN";
}

/// One protocol message, independent from ROS and from the transport used to carry it.
struct Frame
{
  FrameType type{FrameType::kData};
  /// Per topic counter set by the sender. A gap on the server side means lost messages.
  std::uint64_t sequence{0};
  /// UNIX time in nanoseconds at which the robot handed the message to the transmitter.
  std::uint64_t timestamp_ns{0};
  /// ROS topic name, for example "/imu/data". Empty for HELLO and HEARTBEAT.
  std::string topic;
  /// ROS message type, for example "sensor_msgs/msg/Imu". Empty for HELLO and HEARTBEAT.
  std::string message_type;
  /// Message body. For DATA frames this is the CDR serialized ROS message.
  std::vector<std::uint8_t> payload;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__PROTOCOL__FRAME_HPP_
