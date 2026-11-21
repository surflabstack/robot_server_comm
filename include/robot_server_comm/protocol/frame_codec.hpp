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

#ifndef ROBOT_SERVER_COMM__PROTOCOL__FRAME_CODEC_HPP_
#define ROBOT_SERVER_COMM__PROTOCOL__FRAME_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot_server_comm/protocol/frame.hpp"

namespace robot_server_comm
{

// Wire format, version 1. Every integer is big-endian (network byte order).
//
//   offset  size  field
//   0       4     magic "RSCP"
//   4       1     protocol version (1)
//   5       1     frame type (1 DATA, 2 HELLO, 3 HEARTBEAT)
//   6       2     flags (bit 0: body CRC present)
//   8       8     sequence number (per topic)
//   16      8     timestamp, UNIX nanoseconds
//   24      2     topic length T
//   26      2     message type length M
//   28      4     payload length P
//   32      4     body CRC-32 over topic + message type + payload (0 when the flag is off)
//   36      4     header CRC-32 over bytes 0..35
//   40      T     topic (UTF-8, no terminator)
//   40+T    M     message type (UTF-8, no terminator)
//   40+T+M  P     payload (CDR serialized ROS message for DATA frames)

/// Builds the header of a frame: fixed part followed by topic and message type.
///
/// The payload is not copied. The caller sends the returned bytes and then
/// `frame.payload` (gather write), so large point clouds or images are never
/// duplicated just to be framed.
///
/// \param frame frame to encode
/// \param with_body_crc when true the body CRC is computed and flagged
/// \throws std::invalid_argument when a name or the payload exceeds the protocol limits
std::vector<std::uint8_t> encode_frame_header(const Frame & frame, bool with_body_crc = true);

/// Builds a complete frame (header and payload) in one contiguous buffer.
/// Convenient for tests and small frames.
std::vector<std::uint8_t> encode_frame(const Frame & frame, bool with_body_crc = true);

/// Result of one decoding step.
enum class DecodeStatus
{
  kNeedMoreData,  ///< No complete frame is buffered yet.
  kFrameReady,    ///< A frame was written to the output argument.
  kError,         ///< The stream is corrupt, see FrameDecoder::error().
};

/// Incremental decoder for a byte stream (TCP delivers bytes, not messages).
///
/// Bytes are pushed with feed() in chunks of any size and frames are pulled with
/// next() until it returns kNeedMoreData. After kError the stream cannot be
/// trusted any more: the owner should close the connection and call reset().
class FrameDecoder
{
public:
  explicit FrameDecoder(std::size_t max_payload_size = kDefaultMaxPayloadSize);

  /// Appends received bytes to the internal buffer.
  void feed(const std::uint8_t * data, std::size_t size);

  /// Extracts the next complete frame if there is one.
  DecodeStatus next(Frame & frame);

  /// Description of the last error (empty when there was none).
  const std::string & error() const {return error_;}

  /// Number of bytes received and not consumed yet.
  std::size_t buffered_bytes() const {return buffer_.size() - read_offset_;}

  /// Drops buffered bytes and clears the error state.
  void reset();

private:
  DecodeStatus fail(const std::string & reason);
  void compact();

  std::size_t max_payload_size_;
  std::vector<std::uint8_t> buffer_;
  std::size_t read_offset_{0};
  bool failed_{false};
  std::string error_;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__PROTOCOL__FRAME_CODEC_HPP_
