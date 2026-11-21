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

#ifndef ROBOT_SERVER_COMM__PROTOCOL__BYTE_ORDER_HPP_
#define ROBOT_SERVER_COMM__PROTOCOL__BYTE_ORDER_HPP_

#include <cstdint>

namespace robot_server_comm
{

// Big-endian (network byte order) readers and writers.
//
// They work byte by byte with shifts, so the result is the same on any CPU
// (x86_64, arm64, big-endian targets) and never depends on struct layout or
// on unaligned memory access.

inline void put_u16(std::uint8_t * out, std::uint16_t value)
{
  out[0] = static_cast<std::uint8_t>(value >> 8);
  out[1] = static_cast<std::uint8_t>(value);
}

inline void put_u32(std::uint8_t * out, std::uint32_t value)
{
  out[0] = static_cast<std::uint8_t>(value >> 24);
  out[1] = static_cast<std::uint8_t>(value >> 16);
  out[2] = static_cast<std::uint8_t>(value >> 8);
  out[3] = static_cast<std::uint8_t>(value);
}

inline void put_u64(std::uint8_t * out, std::uint64_t value)
{
  put_u32(out, static_cast<std::uint32_t>(value >> 32));
  put_u32(out + 4, static_cast<std::uint32_t>(value));
}

inline std::uint16_t get_u16(const std::uint8_t * in)
{
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(in[0]) << 8) | in[1]);
}

inline std::uint32_t get_u32(const std::uint8_t * in)
{
  return (static_cast<std::uint32_t>(in[0]) << 24) |
         (static_cast<std::uint32_t>(in[1]) << 16) |
         (static_cast<std::uint32_t>(in[2]) << 8) |
         static_cast<std::uint32_t>(in[3]);
}

inline std::uint64_t get_u64(const std::uint8_t * in)
{
  return (static_cast<std::uint64_t>(get_u32(in)) << 32) | get_u32(in + 4);
}

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__PROTOCOL__BYTE_ORDER_HPP_
