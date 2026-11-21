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

#ifndef ROBOT_SERVER_COMM__PROTOCOL__CRC32_HPP_
#define ROBOT_SERVER_COMM__PROTOCOL__CRC32_HPP_

#include <cstddef>
#include <cstdint>

namespace robot_server_comm
{

/// CRC-32 (IEEE 802.3, the same polynomial as zlib and Ethernet).
///
/// The function can be chained to checksum several buffers as if they were one:
/// `crc32(b, nb, crc32(a, na))` equals the CRC of `a` followed by `b`.
///
/// \param data bytes to checksum (may be null when size is 0)
/// \param size number of bytes
/// \param seed CRC of the bytes that came before, 0 for the first buffer
std::uint32_t crc32(const std::uint8_t * data, std::size_t size, std::uint32_t seed = 0);

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__PROTOCOL__CRC32_HPP_
