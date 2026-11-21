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

#include "robot_server_comm/protocol/frame_codec.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_server_comm/protocol/byte_order.hpp"
#include "robot_server_comm/protocol/crc32.hpp"

namespace robot_server_comm
{

namespace
{

// Offsets of the fields inside the fixed header.
constexpr std::size_t kOffsetVersion = 4;
constexpr std::size_t kOffsetType = 5;
constexpr std::size_t kOffsetFlags = 6;
constexpr std::size_t kOffsetSequence = 8;
constexpr std::size_t kOffsetTimestamp = 16;
constexpr std::size_t kOffsetTopicLength = 24;
constexpr std::size_t kOffsetTypeLength = 26;
constexpr std::size_t kOffsetPayloadLength = 28;
constexpr std::size_t kOffsetBodyCrc = 32;
constexpr std::size_t kOffsetHeaderCrc = 36;

// Consumed bytes are removed from the front of the buffer once they exceed this size.
constexpr std::size_t kCompactThreshold = 64u * 1024u;

const std::uint8_t * as_bytes(const std::string & text)
{
  return reinterpret_cast<const std::uint8_t *>(text.data());
}

}  // namespace

std::vector<std::uint8_t> encode_frame_header(const Frame & frame, bool with_body_crc)
{
  if (frame.topic.size() > kMaxNameLength) {
    throw std::invalid_argument("topic name is longer than the protocol limit");
  }
  if (frame.message_type.size() > kMaxNameLength) {
    throw std::invalid_argument("message type name is longer than the protocol limit");
  }
  if (frame.payload.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("payload does not fit in a 32 bit length field");
  }

  std::vector<std::uint8_t> out(
    kFixedHeaderSize + frame.topic.size() + frame.message_type.size());
  std::uint8_t * header = out.data();

  std::memcpy(header, kFrameMagic, sizeof(kFrameMagic));
  header[kOffsetVersion] = kProtocolVersion;
  header[kOffsetType] = static_cast<std::uint8_t>(frame.type);
  put_u16(header + kOffsetFlags, with_body_crc ? kFlagBodyCrc : std::uint16_t{0});
  put_u64(header + kOffsetSequence, frame.sequence);
  put_u64(header + kOffsetTimestamp, frame.timestamp_ns);
  put_u16(header + kOffsetTopicLength, static_cast<std::uint16_t>(frame.topic.size()));
  put_u16(header + kOffsetTypeLength, static_cast<std::uint16_t>(frame.message_type.size()));
  put_u32(header + kOffsetPayloadLength, static_cast<std::uint32_t>(frame.payload.size()));

  std::uint32_t body_crc = 0;
  if (with_body_crc) {
    body_crc = crc32(as_bytes(frame.topic), frame.topic.size());
    body_crc = crc32(as_bytes(frame.message_type), frame.message_type.size(), body_crc);
    body_crc = crc32(frame.payload.data(), frame.payload.size(), body_crc);
  }
  put_u32(header + kOffsetBodyCrc, body_crc);
  put_u32(header + kOffsetHeaderCrc, crc32(header, kOffsetHeaderCrc));

  std::uint8_t * names = header + kFixedHeaderSize;
  if (!frame.topic.empty()) {
    std::memcpy(names, frame.topic.data(), frame.topic.size());
  }
  if (!frame.message_type.empty()) {
    std::memcpy(names + frame.topic.size(), frame.message_type.data(), frame.message_type.size());
  }
  return out;
}

std::vector<std::uint8_t> encode_frame(const Frame & frame, bool with_body_crc)
{
  std::vector<std::uint8_t> out = encode_frame_header(frame, with_body_crc);
  out.insert(out.end(), frame.payload.begin(), frame.payload.end());
  return out;
}

FrameDecoder::FrameDecoder(std::size_t max_payload_size)
: max_payload_size_(max_payload_size)
{
}

void FrameDecoder::feed(const std::uint8_t * data, std::size_t size)
{
  if (failed_ || data == nullptr || size == 0) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + size);
}

DecodeStatus FrameDecoder::next(Frame & frame)
{
  if (failed_) {
    return DecodeStatus::kError;
  }
  if (buffered_bytes() < kFixedHeaderSize) {
    return DecodeStatus::kNeedMoreData;
  }

  const std::uint8_t * header = buffer_.data() + read_offset_;

  // The header is validated before waiting for the body, so a corrupt length
  // field can never make the receiver wait for (or allocate) gigabytes.
  if (std::memcmp(header, kFrameMagic, sizeof(kFrameMagic)) != 0) {
    return fail("bad magic bytes, the stream is not a RSCP stream or is out of sync");
  }
  if (get_u32(header + kOffsetHeaderCrc) != crc32(header, kOffsetHeaderCrc)) {
    return fail("header checksum mismatch");
  }
  if (header[kOffsetVersion] != kProtocolVersion) {
    return fail(
      "unsupported protocol version " + std::to_string(header[kOffsetVersion]) +
      " (this build speaks version " + std::to_string(kProtocolVersion) + ")");
  }
  if (!is_valid_frame_type(header[kOffsetType])) {
    return fail("unknown frame type " + std::to_string(header[kOffsetType]));
  }

  const std::uint16_t flags = get_u16(header + kOffsetFlags);
  const std::size_t topic_length = get_u16(header + kOffsetTopicLength);
  const std::size_t type_length = get_u16(header + kOffsetTypeLength);
  const std::size_t payload_length = get_u32(header + kOffsetPayloadLength);

  if (topic_length > kMaxNameLength || type_length > kMaxNameLength) {
    return fail("topic or message type name is longer than the protocol limit");
  }
  if (payload_length > max_payload_size_) {
    return fail(
      "payload of " + std::to_string(payload_length) + " bytes exceeds the configured limit of " +
      std::to_string(max_payload_size_) + " bytes");
  }

  const std::size_t body_length = topic_length + type_length + payload_length;
  if (buffered_bytes() < kFixedHeaderSize + body_length) {
    return DecodeStatus::kNeedMoreData;
  }

  const std::uint8_t * body = header + kFixedHeaderSize;
  if ((flags & kFlagBodyCrc) != 0 &&
    get_u32(header + kOffsetBodyCrc) != crc32(body, body_length))
  {
    return fail("body checksum mismatch");
  }

  frame.type = static_cast<FrameType>(header[kOffsetType]);
  frame.sequence = get_u64(header + kOffsetSequence);
  frame.timestamp_ns = get_u64(header + kOffsetTimestamp);
  frame.topic.assign(reinterpret_cast<const char *>(body), topic_length);
  frame.message_type.assign(reinterpret_cast<const char *>(body + topic_length), type_length);
  const std::uint8_t * payload = body + topic_length + type_length;
  frame.payload.assign(payload, payload + payload_length);

  read_offset_ += kFixedHeaderSize + body_length;
  compact();
  return DecodeStatus::kFrameReady;
}

void FrameDecoder::reset()
{
  buffer_.clear();
  read_offset_ = 0;
  failed_ = false;
  error_.clear();
}

DecodeStatus FrameDecoder::fail(const std::string & reason)
{
  failed_ = true;
  error_ = reason;
  buffer_.clear();
  read_offset_ = 0;
  return DecodeStatus::kError;
}

void FrameDecoder::compact()
{
  if (read_offset_ == buffer_.size()) {
    buffer_.clear();
    read_offset_ = 0;
  } else if (read_offset_ >= kCompactThreshold) {
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(read_offset_));
    read_offset_ = 0;
  }
}

}  // namespace robot_server_comm
