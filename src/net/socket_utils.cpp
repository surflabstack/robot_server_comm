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

#include "robot_server_comm/net/socket_utils.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>

#include <string>
#include <system_error>

namespace robot_server_comm
{

void UniqueFd::reset(int fd)
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
  fd_ = fd;
}

bool set_non_blocking(int fd)
{
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

std::string errno_to_string(int error_number)
{
  return std::generic_category().message(error_number);
}

std::string format_address(const sockaddr * address, socklen_t length)
{
  char host[NI_MAXHOST] = {0};
  char service[NI_MAXSERV] = {0};
  const int rc = ::getnameinfo(
    address, length, host, sizeof(host), service, sizeof(service),
    NI_NUMERICHOST | NI_NUMERICSERV);
  if (rc != 0) {
    return "unknown";
  }
  if (address->sa_family == AF_INET6) {
    return "[" + std::string(host) + "]:" + service;
  }
  return std::string(host) + ":" + service;
}

}  // namespace robot_server_comm
