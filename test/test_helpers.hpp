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

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

#include "robot_server_comm/net/socket_utils.hpp"

namespace robot_server_comm
{
namespace test_support
{

/// Polls `condition` until it is true or `timeout` expires. Returns the final value.
inline bool wait_until(
  const std::function<bool()> & condition,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return condition();
}

/// Minimal TCP listener on 127.0.0.1 with a port chosen by the system.
class LoopbackListener
{
public:
  LoopbackListener()
  {
    socket_.reset(::socket(AF_INET, SOCK_STREAM, 0));
    const int enable = 1;
    ::setsockopt(socket_.get(), SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    ::bind(socket_.get(), reinterpret_cast<const sockaddr *>(&address), sizeof(address));
    ::listen(socket_.get(), 8);
    socklen_t length = sizeof(address);
    ::getsockname(socket_.get(), reinterpret_cast<sockaddr *>(&address), &length);
    port_ = ntohs(address.sin_port);
  }

  std::uint16_t port() const {return port_;}

  /// Waits for one client. The returned descriptor is invalid on timeout.
  UniqueFd accept_client(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
  {
    pollfd descriptor{socket_.get(), POLLIN, 0};
    if (::poll(&descriptor, 1, static_cast<int>(timeout.count())) <= 0) {
      return UniqueFd();
    }
    return UniqueFd(::accept(socket_.get(), nullptr, nullptr));
  }

  /// Stops listening, later connection attempts are refused.
  void close() {socket_.reset();}

private:
  UniqueFd socket_;
  std::uint16_t port_{0};
};

/// Reads exactly `size` bytes. Returns false on end of stream, error or timeout.
inline bool read_exact(
  int fd, std::uint8_t * data, std::size_t size,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(10000))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t done = 0;
  while (done < size) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      return false;
    }
    pollfd descriptor{fd, POLLIN, 0};
    if (::poll(&descriptor, 1, static_cast<int>(remaining.count())) <= 0) {
      return false;
    }
    const ssize_t received = ::recv(fd, data + done, size - done, 0);
    if (received <= 0) {
      return false;
    }
    done += static_cast<std::size_t>(received);
  }
  return true;
}

}  // namespace test_support
}  // namespace robot_server_comm

#endif  // TEST_HELPERS_HPP_
