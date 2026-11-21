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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_server_comm/protocol/byte_order.hpp"
#include "robot_server_comm/protocol/crc32.hpp"
#include "robot_server_comm/protocol/frame.hpp"
#include "robot_server_comm/protocol/frame_codec.hpp"

namespace robot_server_comm
{

namespace
{

Frame make_data_frame(std::size_t payload_size = 64)
{
  Frame frame;
  frame.type = FrameType::kData;
  frame.sequence = 42;
  frame.timestamp_ns = 1556262001123456789ull;
  frame.topic = "/imu/data";
  frame.message_type = "sensor_msgs/msg/Imu";
  frame.payload.resize(payload_size);
  for (std::size_t i = 0; i < payload_size; ++i) {
    frame.payload[i] = static_cast<std::uint8_t>(i * 7 + 3);
  }
  return frame;
}

void expect_same(const Frame & expected, const Frame & actual)
{
  EXPECT_EQ(expected.type, actual.type);
  EXPECT_EQ(expected.sequence, actual.sequence);
  EXPECT_EQ(expected.timestamp_ns, actual.timestamp_ns);
  EXPECT_EQ(expected.topic, actual.topic);
  EXPECT_EQ(expected.message_type, actual.message_type);
  EXPECT_EQ(expected.payload, actual.payload);
}

// Rewrites the header checksum after a test tampered with a header field on purpose.
void refresh_header_crc(std::vector<std::uint8_t> & bytes)
{
  put_u32(bytes.data() + 36, crc32(bytes.data(), 36));
}

}  // namespace

TEST(Crc32, MatchesTheStandardCheckValue)
{
  const std::string text = "123456789";
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(text.data());
  EXPECT_EQ(0xCBF43926u, crc32(bytes, text.size()));
}

TEST(Crc32, CanBeChainedOverSeveralBuffers)
{
  const std::string text = "123456789";
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(text.data());
  const std::uint32_t first = crc32(bytes, 4);
  EXPECT_EQ(crc32(bytes, text.size()), crc32(bytes + 4, text.size() - 4, first));
  EXPECT_EQ(0u, crc32(nullptr, 0));
}

TEST(Crc32, FastImplementationMatchesTheBitByBitDefinition)
{
  // Reference: the textbook algorithm, one bit at a time.
  const auto reference = [](const std::vector<std::uint8_t> & data) {
      std::uint32_t crc = 0xFFFFFFFFu;
      for (const std::uint8_t byte : data) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) {
          crc = (crc & 1u) ? (0xEDB88320u ^ (crc >> 1)) : (crc >> 1);
        }
      }
      return ~crc;
    };

  // Every length from 0 to 300 covers all the alignments of the 8 byte fast path.
  std::vector<std::uint8_t> data;
  std::uint32_t state = 12345;
  for (std::size_t length = 0; length <= 300; ++length) {
    EXPECT_EQ(reference(data), crc32(data.data(), data.size())) << "length " << length;
    state = state * 1664525u + 1013904223u;
    data.push_back(static_cast<std::uint8_t>(state >> 24));
  }
}

TEST(ByteOrder, WritesBigEndianOnAnyHost)
{
  std::uint8_t buffer[8] = {};
  put_u32(buffer, 0x01020304u);
  EXPECT_EQ(0x01, buffer[0]);
  EXPECT_EQ(0x04, buffer[3]);
  EXPECT_EQ(0x01020304u, get_u32(buffer));

  put_u64(buffer, 0x0102030405060708ull);
  EXPECT_EQ(0x01, buffer[0]);
  EXPECT_EQ(0x08, buffer[7]);
  EXPECT_EQ(0x0102030405060708ull, get_u64(buffer));

  put_u16(buffer, 0xABCD);
  EXPECT_EQ(0xAB, buffer[0]);
  EXPECT_EQ(0xABCD, get_u16(buffer));
}

TEST(FrameCodec, HeaderLayoutIsTheDocumentedOne)
{
  const Frame frame = make_data_frame(5);
  const std::vector<std::uint8_t> bytes = encode_frame(frame);

  ASSERT_EQ(kFixedHeaderSize + frame.topic.size() + frame.message_type.size() + 5, bytes.size());
  EXPECT_EQ('R', bytes[0]);
  EXPECT_EQ('S', bytes[1]);
  EXPECT_EQ('C', bytes[2]);
  EXPECT_EQ('P', bytes[3]);
  EXPECT_EQ(kProtocolVersion, bytes[4]);
  EXPECT_EQ(1, bytes[5]);
  EXPECT_EQ(kFlagBodyCrc, get_u16(&bytes[6]));
  EXPECT_EQ(42u, get_u64(&bytes[8]));
  EXPECT_EQ(1556262001123456789ull, get_u64(&bytes[16]));
  EXPECT_EQ(frame.topic.size(), get_u16(&bytes[24]));
  EXPECT_EQ(frame.message_type.size(), get_u16(&bytes[26]));
  EXPECT_EQ(5u, get_u32(&bytes[28]));
  EXPECT_EQ(crc32(&bytes[40], bytes.size() - 40), get_u32(&bytes[32]));
  EXPECT_EQ(crc32(bytes.data(), 36), get_u32(&bytes[36]));
  EXPECT_EQ("/imu/data", std::string(bytes.begin() + 40, bytes.begin() + 49));
}

TEST(FrameCodec, RoundTripsADataFrame)
{
  const Frame original = make_data_frame(1000);
  const std::vector<std::uint8_t> bytes = encode_frame(original);

  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  ASSERT_EQ(DecodeStatus::kFrameReady, decoder.next(decoded));
  expect_same(original, decoded);
  EXPECT_EQ(DecodeStatus::kNeedMoreData, decoder.next(decoded));
  EXPECT_EQ(0u, decoder.buffered_bytes());
}

TEST(FrameCodec, RoundTripsFramesWithoutBody)
{
  Frame heartbeat;
  heartbeat.type = FrameType::kHeartbeat;
  heartbeat.timestamp_ns = 7;
  const std::vector<std::uint8_t> bytes = encode_frame(heartbeat);
  EXPECT_EQ(kFixedHeaderSize, bytes.size());

  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  ASSERT_EQ(DecodeStatus::kFrameReady, decoder.next(decoded));
  expect_same(heartbeat, decoded);
}

TEST(FrameCodec, DecodesAStreamDeliveredOneByteAtATime)
{
  const Frame original = make_data_frame(300);
  const std::vector<std::uint8_t> bytes = encode_frame(original);

  FrameDecoder decoder;
  Frame decoded;
  for (std::size_t i = 0; i + 1 < bytes.size(); ++i) {
    decoder.feed(&bytes[i], 1);
    ASSERT_EQ(DecodeStatus::kNeedMoreData, decoder.next(decoded)) << "at byte " << i;
  }
  decoder.feed(&bytes.back(), 1);
  ASSERT_EQ(DecodeStatus::kFrameReady, decoder.next(decoded));
  expect_same(original, decoded);
}

TEST(FrameCodec, DecodesSeveralFramesReceivedInOneChunk)
{
  std::vector<std::uint8_t> stream;
  std::vector<Frame> originals;
  for (std::uint64_t i = 0; i < 50; ++i) {
    Frame frame = make_data_frame(static_cast<std::size_t>(10 + i * 37));
    frame.sequence = i;
    const std::vector<std::uint8_t> bytes = encode_frame(frame);
    stream.insert(stream.end(), bytes.begin(), bytes.end());
    originals.push_back(frame);
  }

  FrameDecoder decoder;
  // Odd chunk size on purpose: frame boundaries never line up with chunk boundaries.
  const std::size_t chunk = 1013;
  std::vector<Frame> decoded;
  for (std::size_t offset = 0; offset < stream.size(); offset += chunk) {
    const std::size_t size = std::min(chunk, stream.size() - offset);
    decoder.feed(&stream[offset], size);
    Frame frame;
    DecodeStatus status;
    while ((status = decoder.next(frame)) == DecodeStatus::kFrameReady) {
      decoded.push_back(frame);
    }
    ASSERT_EQ(DecodeStatus::kNeedMoreData, status);
  }
  ASSERT_EQ(originals.size(), decoded.size());
  for (std::size_t i = 0; i < originals.size(); ++i) {
    expect_same(originals[i], decoded[i]);
  }
}

TEST(FrameCodec, HandlesALargePayload)
{
  // About the size of one Velodyne HDL-32E scan.
  const Frame original = make_data_frame(1500000);
  const std::vector<std::uint8_t> bytes = encode_frame(original);
  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  ASSERT_EQ(DecodeStatus::kFrameReady, decoder.next(decoded));
  expect_same(original, decoded);
}

TEST(FrameCodec, RejectsAStreamThatDoesNotStartWithTheMagic)
{
  std::vector<std::uint8_t> bytes = encode_frame(make_data_frame());
  bytes[0] = 'X';
  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  EXPECT_EQ(DecodeStatus::kError, decoder.next(decoded));
  EXPECT_NE(std::string::npos, decoder.error().find("magic"));
}

TEST(FrameCodec, RejectsACorruptedHeader)
{
  std::vector<std::uint8_t> bytes = encode_frame(make_data_frame());
  bytes[30] ^= 0x40;  // inside the payload length field
  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  EXPECT_EQ(DecodeStatus::kError, decoder.next(decoded));
  EXPECT_NE(std::string::npos, decoder.error().find("header checksum"));
}

TEST(FrameCodec, RejectsACorruptedPayload)
{
  std::vector<std::uint8_t> bytes = encode_frame(make_data_frame());
  bytes.back() ^= 0x01;
  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  EXPECT_EQ(DecodeStatus::kError, decoder.next(decoded));
  EXPECT_NE(std::string::npos, decoder.error().find("body checksum"));
}

TEST(FrameCodec, BodyChecksumIsOptional)
{
  const Frame original = make_data_frame();
  std::vector<std::uint8_t> bytes = encode_frame(original, false);
  EXPECT_EQ(0u, get_u16(&bytes[6]));
  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  ASSERT_EQ(DecodeStatus::kFrameReady, decoder.next(decoded));
  expect_same(original, decoded);
}

TEST(FrameCodec, RejectsAnotherProtocolVersion)
{
  std::vector<std::uint8_t> bytes = encode_frame(make_data_frame());
  bytes[4] = kProtocolVersion + 1;
  refresh_header_crc(bytes);
  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  EXPECT_EQ(DecodeStatus::kError, decoder.next(decoded));
  EXPECT_NE(std::string::npos, decoder.error().find("version"));
}

TEST(FrameCodec, RejectsAnUnknownFrameType)
{
  std::vector<std::uint8_t> bytes = encode_frame(make_data_frame());
  bytes[5] = 99;
  refresh_header_crc(bytes);
  FrameDecoder decoder;
  decoder.feed(bytes.data(), bytes.size());
  Frame decoded;
  EXPECT_EQ(DecodeStatus::kError, decoder.next(decoded));
  EXPECT_NE(std::string::npos, decoder.error().find("frame type"));
}

TEST(FrameCodec, RejectsAPayloadLargerThanTheConfiguredLimit)
{
  const std::vector<std::uint8_t> bytes = encode_frame(make_data_frame(2048));
  FrameDecoder decoder(1024);
  // Only the header is needed to refuse: the receiver never buffers the oversized body.
  decoder.feed(bytes.data(), kFixedHeaderSize);
  Frame decoded;
  EXPECT_EQ(DecodeStatus::kError, decoder.next(decoded));
  EXPECT_NE(std::string::npos, decoder.error().find("exceeds"));
}

TEST(FrameCodec, StaysInErrorUntilReset)
{
  std::vector<std::uint8_t> bad = encode_frame(make_data_frame());
  bad[0] = 0;
  const Frame original = make_data_frame();
  const std::vector<std::uint8_t> good = encode_frame(original);

  FrameDecoder decoder;
  decoder.feed(bad.data(), bad.size());
  Frame decoded;
  ASSERT_EQ(DecodeStatus::kError, decoder.next(decoded));
  decoder.feed(good.data(), good.size());
  EXPECT_EQ(DecodeStatus::kError, decoder.next(decoded));

  decoder.reset();
  EXPECT_TRUE(decoder.error().empty());
  decoder.feed(good.data(), good.size());
  ASSERT_EQ(DecodeStatus::kFrameReady, decoder.next(decoded));
  expect_same(original, decoded);
}

TEST(FrameCodec, EncoderRefusesNamesLongerThanTheLimit)
{
  Frame frame = make_data_frame();
  frame.topic.assign(kMaxNameLength + 1, 'a');
  EXPECT_THROW(encode_frame(frame), std::invalid_argument);

  frame = make_data_frame();
  frame.message_type.assign(kMaxNameLength + 1, 'a');
  EXPECT_THROW(encode_frame(frame), std::invalid_argument);
}

}  // namespace robot_server_comm
