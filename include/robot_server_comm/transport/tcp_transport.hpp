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

#ifndef ROBOT_SERVER_COMM__TRANSPORT__TCP_TRANSPORT_HPP_
#define ROBOT_SERVER_COMM__TRANSPORT__TCP_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "robot_server_comm/net/socket_utils.hpp"
#include "robot_server_comm/transport/transport.hpp"

namespace robot_server_comm
{

/// Settings of a TCP client link.
struct TcpTransportConfig
{
  std::string host{"127.0.0.1"};
  std::uint16_t port{9000};
  /// Longest time to wait for the TCP handshake.
  std::chrono::milliseconds connect_timeout{2000};
  /// Longest time a send may stay blocked by a full socket buffer (slow or stalled server).
  std::chrono::milliseconds send_timeout{2000};
  /// Disable Nagle's algorithm so small sensor messages leave immediately.
  bool tcp_no_delay{true};
  /// Let the kernel probe an idle connection to detect a dead peer.
  bool keep_alive{true};
};

/// TCP client transport built on POSIX sockets (IPv4 and IPv6).
///
/// The socket is non-blocking and every wait goes through poll() with a
/// timeout, so no call can hang forever when the server or the network stalls.
class TcpTransport : public Transport
{
public:
  explicit TcpTransport(TcpTransportConfig config);
  ~TcpTransport() override;

  bool connect() override;
  void disconnect() override;
  bool is_connected() override;
  bool send(const std::vector<ConstBuffer> & buffers) override;
  std::string last_error() const override {return last_error_;}
  std::string description() const override;

private:
  bool connect_to(const sockaddr * address, socklen_t length);
  void configure_socket(int fd) const;
  bool wait_writable(std::chrono::milliseconds timeout);
  bool fail(const std::string & reason);

  TcpTransportConfig config_;
  UniqueFd socket_;
  std::string last_error_;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__TRANSPORT__TCP_TRANSPORT_HPP_
