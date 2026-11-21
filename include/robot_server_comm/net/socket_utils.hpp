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

#ifndef ROBOT_SERVER_COMM__NET__SOCKET_UTILS_HPP_
#define ROBOT_SERVER_COMM__NET__SOCKET_UTILS_HPP_

#include <netdb.h>
#include <sys/socket.h>

#include <memory>
#include <string>

namespace robot_server_comm
{

/// Owns a POSIX file descriptor and closes it when destroyed (RAII).
class UniqueFd
{
public:
  UniqueFd() = default;
  explicit UniqueFd(int fd)
  : fd_(fd) {}
  ~UniqueFd() {reset();}

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;

  UniqueFd(UniqueFd && other) noexcept
  : fd_(other.release()) {}

  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  int get() const {return fd_;}
  bool valid() const {return fd_ >= 0;}

  /// Gives up ownership without closing.
  int release()
  {
    const int fd = fd_;
    fd_ = -1;
    return fd;
  }

  /// Closes the current descriptor and takes ownership of a new one.
  void reset(int fd = -1);

private:
  int fd_{-1};
};

/// Releases the list returned by getaddrinfo() when it goes out of scope.
struct AddrInfoDeleter
{
  void operator()(addrinfo * list) const {::freeaddrinfo(list);}
};
using AddrInfoPtr = std::unique_ptr<addrinfo, AddrInfoDeleter>;

/// Switches a descriptor to non-blocking mode. Returns false on failure.
bool set_non_blocking(int fd);

/// Text for an errno value, thread safe.
std::string errno_to_string(int error_number);

/// Formats a socket address as "ip:port" (IPv4) or "[ip]:port" (IPv6).
std::string format_address(const sockaddr * address, socklen_t length);

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__NET__SOCKET_UTILS_HPP_
