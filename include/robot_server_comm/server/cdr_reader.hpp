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

#ifndef ROBOT_SERVER_COMM__SERVER__CDR_READER_HPP_
#define ROBOT_SERVER_COMM__SERVER__CDR_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace robot_server_comm
{

/// Error raised when a CDR buffer is truncated or malformed.
class CdrError : public std::runtime_error
{
public:
  explicit CdrError(const std::string & what)
  : std::runtime_error(what) {}
};

/// Minimal reader for CDR, the serialization format ROS 2 uses on the wire.
///
/// It lets the server look inside a message without any ROS installation:
/// only the rules needed for plain messages are implemented (4 byte
/// encapsulation header, both byte orders, alignment of primitives to their
/// own size, strings and sequences prefixed by a 32 bit length).
/// Every read is bounds checked and throws CdrError instead of reading past
/// the end of the buffer.
class CdrReader
{
public:
  CdrReader(const std::uint8_t * data, std::size_t size)
  {
    if (data == nullptr || size < kEncapsulationSize) {
      throw CdrError("buffer too small for a CDR encapsulation header");
    }
    // Representation identifier: 0x0000 big-endian, 0x0001 little-endian
    // (0x0002 and 0x0003 are the parameter list variants with the same byte orders).
    if (data[0] != 0x00 || data[1] > 0x03) {
      throw CdrError("unsupported CDR representation identifier");
    }
    little_endian_ = (data[1] & 0x01) != 0;
    // Alignment is counted from the first byte after the encapsulation header.
    data_ = data + kEncapsulationSize;
    size_ = size - kEncapsulationSize;
  }

  /// Reads one arithmetic value (integers, floats, bool as uint8).
  template<typename ValueT>
  ValueT read()
  {
    static_assert(std::is_arithmetic<ValueT>::value, "read() only handles arithmetic types");
    align(sizeof(ValueT));
    require(sizeof(ValueT));
    std::uint8_t raw[sizeof(ValueT)];
    for (std::size_t i = 0; i < sizeof(ValueT); ++i) {
      // Reorder the bytes when the sender and this machine disagree on byte order.
      const std::size_t source = (little_endian_ == host_is_little_endian()) ?
        i : sizeof(ValueT) - 1 - i;
      raw[i] = data_[offset_ + source];
    }
    offset_ += sizeof(ValueT);
    ValueT value;
    std::memcpy(&value, raw, sizeof(ValueT));
    return value;
  }

  /// Reads a string (32 bit length that includes the terminating zero).
  std::string read_string()
  {
    const std::uint32_t length = read<std::uint32_t>();
    require(length);
    std::string text(reinterpret_cast<const char *>(data_ + offset_), length);
    offset_ += length;
    while (!text.empty() && text.back() == '\0') {
      text.pop_back();
    }
    return text;
  }

  /// Skips `count` consecutive values of type ValueT (fixed size array or sequence body).
  template<typename ValueT>
  void skip(std::size_t count)
  {
    if (count == 0) {
      return;
    }
    align(sizeof(ValueT));
    if (count > (size_ - offset_) / sizeof(ValueT)) {
      throw CdrError("CDR buffer is truncated");
    }
    offset_ += count * sizeof(ValueT);
  }

  /// Bytes left after the read position.
  std::size_t remaining() const {return size_ - offset_;}

private:
  static constexpr std::size_t kEncapsulationSize = 4;

  static bool host_is_little_endian()
  {
    const std::uint16_t probe = 1;
    std::uint8_t first_byte;
    std::memcpy(&first_byte, &probe, 1);
    return first_byte == 1;
  }

  void align(std::size_t alignment)
  {
    const std::size_t misalignment = offset_ % alignment;
    if (misalignment != 0) {
      require(alignment - misalignment);
      offset_ += alignment - misalignment;
    }
  }

  void require(std::size_t count) const
  {
    if (count > size_ - offset_) {
      throw CdrError("CDR buffer is truncated");
    }
  }

  const std::uint8_t * data_{nullptr};
  std::size_t size_{0};
  std::size_t offset_{0};
  bool little_endian_{true};
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__SERVER__CDR_READER_HPP_
