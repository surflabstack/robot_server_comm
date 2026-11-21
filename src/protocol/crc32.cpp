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

#include "robot_server_comm/protocol/crc32.hpp"

#include <array>
#include <cstddef>

namespace robot_server_comm
{

namespace
{

constexpr std::uint32_t kPolynomial = 0xEDB88320u;
constexpr std::size_t kSlices = 8;

using Tables = std::array<std::array<std::uint32_t, 256>, kSlices>;

// Lookup tables for the "slicing by 8" method: 8 input bytes are folded into
// the CRC per step instead of 1, which is several times faster on large
// payloads (point clouds, images) and gives the same result as the classic
// byte by byte algorithm.
constexpr Tables make_tables()
{
  Tables tables{};
  for (std::uint32_t i = 0; i < 256; ++i) {
    std::uint32_t value = i;
    for (int bit = 0; bit < 8; ++bit) {
      value = (value & 1u) ? (kPolynomial ^ (value >> 1)) : (value >> 1);
    }
    tables[0][i] = value;
  }
  for (std::size_t slice = 1; slice < kSlices; ++slice) {
    for (std::size_t i = 0; i < 256; ++i) {
      const std::uint32_t previous = tables[slice - 1][i];
      tables[slice][i] = (previous >> 8) ^ tables[0][previous & 0xFFu];
    }
  }
  return tables;
}

constexpr Tables kTables = make_tables();

// Assembles 4 bytes into a word with explicit shifts. Unlike a memory cast it
// gives the same value on little-endian and big-endian CPUs and never performs
// an unaligned access.
inline std::uint32_t load_le32(const std::uint8_t * bytes)
{
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8) |
         (static_cast<std::uint32_t>(bytes[2]) << 16) |
         (static_cast<std::uint32_t>(bytes[3]) << 24);
}

}  // namespace

std::uint32_t crc32(const std::uint8_t * data, std::size_t size, std::uint32_t seed)
{
  std::uint32_t crc = ~seed;
  while (size >= kSlices) {
    const std::uint32_t low = load_le32(data) ^ crc;
    const std::uint32_t high = load_le32(data + 4);
    crc = kTables[7][low & 0xFFu] ^
      kTables[6][(low >> 8) & 0xFFu] ^
      kTables[5][(low >> 16) & 0xFFu] ^
      kTables[4][low >> 24] ^
      kTables[3][high & 0xFFu] ^
      kTables[2][(high >> 8) & 0xFFu] ^
      kTables[1][(high >> 16) & 0xFFu] ^
      kTables[0][high >> 24];
    data += kSlices;
    size -= kSlices;
  }
  for (std::size_t i = 0; i < size; ++i) {
    crc = kTables[0][(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
  }
  return ~crc;
}

}  // namespace robot_server_comm
