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

#ifndef ROBOT_SERVER_COMM__SERVER__FRAME_SERVER_HPP_
#define ROBOT_SERVER_COMM__SERVER__FRAME_SERVER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <list>
#include <string>
#include <vector>

#include "robot_server_comm/net/socket_utils.hpp"
#include "robot_server_comm/protocol/frame.hpp"
#include "robot_server_comm/protocol/frame_codec.hpp"

namespace robot_server_comm
{

/// Settings of the local server.
struct FrameServerConfig
{
  /// Address to listen on. The default only accepts connections from this machine.
  std::string bind_address{"127.0.0.1"};
  /// Port to listen on. 0 lets the system pick a free port (used by the tests).
  std::uint16_t port{9000};
  /// Largest payload accepted from a client.
  std::size_t max_payload_size{kDefaultMaxPayloadSize};
  /// A client that stays silent for this long (no data, no heartbeat) is disconnected.
  std::chrono::milliseconds client_timeout{5000};
  /// Largest number of robots connected at the same time.
  std::size_t max_clients{16};
};

/// What the server knows about one connected robot.
struct ClientInfo
{
  std::uint64_t id{0};      ///< Connection number, unique for the life of the server.
  std::string address;      ///< Remote "ip:port".
  std::string robot_id;     ///< Name received in the HELLO frame (empty before it arrives).
};

/// Callbacks through which the server reports what happens.
struct FrameServerHandlers
{
  std::function<void(const ClientInfo &)> on_connect;
  std::function<void(const ClientInfo &, const Frame &)> on_frame;
  std::function<void(const ClientInfo &, const std::string & reason)> on_disconnect;
};

/// Small TCP server that plays the role of the Mission Planner Server.
///
/// It accepts several robots, decodes the frame stream of each one and hands
/// complete frames to the handlers. It is single threaded and driven by
/// poll_once(), which makes it easy to embed in a main loop or in a test.
class FrameServer
{
public:
  FrameServer(FrameServerConfig config, FrameServerHandlers handlers);
  ~FrameServer();

  FrameServer(const FrameServer &) = delete;
  FrameServer & operator=(const FrameServer &) = delete;

  /// Opens the listening socket. Returns false and fills `error` on failure.
  bool start(std::string & error);

  /// Closes every connection and the listening socket.
  void stop();

  /// Waits up to `timeout` for network activity and processes it.
  void poll_once(std::chrono::milliseconds timeout);

  /// Port actually used (useful when the configured port is 0).
  std::uint16_t port() const {return port_;}

  /// Number of connected clients.
  std::size_t client_count() const {return clients_.size();}

private:
  using Clock = std::chrono::steady_clock;

  struct Client
  {
    UniqueFd socket;
    ClientInfo info;
    FrameDecoder decoder;
    Clock::time_point last_activity;
    bool closing{false};
    std::string close_reason;
  };

  void accept_clients();
  void read_from(Client & client);
  void drop_closed_clients();

  FrameServerConfig config_;
  FrameServerHandlers handlers_;
  UniqueFd listener_;
  std::uint16_t port_{0};
  std::uint64_t next_client_id_{1};
  std::list<Client> clients_;
  std::vector<std::uint8_t> read_buffer_;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__SERVER__FRAME_SERVER_HPP_
