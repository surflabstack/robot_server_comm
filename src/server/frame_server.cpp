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

#include "robot_server_comm/server/frame_server.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robot_server_comm
{

namespace
{

constexpr int kListenBacklog = 16;
constexpr std::size_t kReadChunkSize = 256u * 1024u;
// Reads served to one client per poll cycle, so a very fast robot cannot starve the others.
constexpr int kMaxReadsPerCycle = 32;
constexpr std::size_t kMaxRobotIdLength = 128;

// Keeps a robot id printable and short before it is shown in a terminal.
std::string sanitize_robot_id(const std::vector<std::uint8_t> & payload)
{
  std::string robot_id;
  for (const std::uint8_t byte : payload) {
    if (robot_id.size() >= kMaxRobotIdLength) {
      break;
    }
    robot_id.push_back((byte >= 0x20 && byte < 0x7F) ? static_cast<char>(byte) : '?');
  }
  return robot_id;
}

}  // namespace

FrameServer::FrameServer(FrameServerConfig config, FrameServerHandlers handlers)
: config_(std::move(config)), handlers_(std::move(handlers)), read_buffer_(kReadChunkSize)
{
}

FrameServer::~FrameServer()
{
  stop();
}

bool FrameServer::start(std::string & error)
{
  stop();

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE | AI_NUMERICSERV;

  addrinfo * raw_results = nullptr;
  const std::string service = std::to_string(config_.port);
  const int rc = ::getaddrinfo(
    config_.bind_address.c_str(), service.c_str(), &hints, &raw_results);
  if (rc != 0) {
    error = "cannot resolve bind address '" + config_.bind_address + "': " + ::gai_strerror(rc);
    return false;
  }
  const AddrInfoPtr results(raw_results);

  error = "no usable address for '" + config_.bind_address + "'";
  for (const addrinfo * candidate = results.get(); candidate != nullptr;
    candidate = candidate->ai_next)
  {
    UniqueFd fd(::socket(candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol));
    if (!fd.valid()) {
      error = "socket() failed: " + errno_to_string(errno);
      continue;
    }
    const int enable = 1;
    (void)::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    if (::bind(fd.get(), candidate->ai_addr, candidate->ai_addrlen) != 0) {
      error = "cannot bind " + format_address(candidate->ai_addr, candidate->ai_addrlen) + ": " +
        errno_to_string(errno);
      continue;
    }
    if (::listen(fd.get(), kListenBacklog) != 0) {
      error = "listen() failed: " + errno_to_string(errno);
      continue;
    }
    if (!set_non_blocking(fd.get())) {
      error = "cannot switch the listening socket to non-blocking mode";
      continue;
    }

    sockaddr_storage bound{};
    socklen_t bound_length = sizeof(bound);
    if (::getsockname(fd.get(), reinterpret_cast<sockaddr *>(&bound), &bound_length) == 0) {
      if (bound.ss_family == AF_INET) {
        port_ = ntohs(reinterpret_cast<const sockaddr_in *>(&bound)->sin_port);
      } else if (bound.ss_family == AF_INET6) {
        port_ = ntohs(reinterpret_cast<const sockaddr_in6 *>(&bound)->sin6_port);
      }
    }
    listener_ = std::move(fd);
    error.clear();
    return true;
  }
  return false;
}

void FrameServer::stop()
{
  clients_.clear();
  listener_.reset();
}

void FrameServer::poll_once(std::chrono::milliseconds timeout)
{
  if (!listener_.valid()) {
    return;
  }

  std::vector<pollfd> descriptors;
  descriptors.reserve(clients_.size() + 1);
  descriptors.push_back({listener_.get(), POLLIN, 0});
  for (const Client & client : clients_) {
    descriptors.push_back({client.socket.get(), POLLIN, 0});
  }

  const int ready = ::poll(
    descriptors.data(), static_cast<nfds_t>(descriptors.size()), static_cast<int>(timeout.count()));

  if (ready > 0) {
    // Existing clients first: their position in the list matches descriptors[1..].
    std::size_t index = 1;
    for (Client & client : clients_) {
      if ((descriptors[index].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
        read_from(client);
      }
      ++index;
    }
    if ((descriptors[0].revents & POLLIN) != 0) {
      accept_clients();
    }
  }

  const Clock::time_point now = Clock::now();
  for (Client & client : clients_) {
    if (!client.closing && now - client.last_activity > config_.client_timeout) {
      client.closing = true;
      client.close_reason = "no data and no heartbeat for " +
        std::to_string(config_.client_timeout.count()) + " ms";
    }
  }
  drop_closed_clients();
}

void FrameServer::accept_clients()
{
  for (;; ) {
    sockaddr_storage peer{};
    socklen_t peer_length = sizeof(peer);
    UniqueFd fd(::accept(listener_.get(), reinterpret_cast<sockaddr *>(&peer), &peer_length));
    if (!fd.valid()) {
      // EAGAIN means the backlog is empty. Any other error only concerns this attempt.
      return;
    }
    if (clients_.size() >= config_.max_clients || !set_non_blocking(fd.get())) {
      continue;  // The descriptor closes itself, the robot will retry later.
    }

    clients_.emplace_back();
    Client & client = clients_.back();
    client.socket = std::move(fd);
    client.info.id = next_client_id_++;
    client.info.address = format_address(reinterpret_cast<const sockaddr *>(&peer), peer_length);
    client.decoder = FrameDecoder(config_.max_payload_size);
    client.last_activity = Clock::now();
    if (handlers_.on_connect) {
      handlers_.on_connect(client.info);
    }
  }
}

void FrameServer::read_from(Client & client)
{
  std::vector<std::uint8_t> & chunk = read_buffer_;
  for (int reads = 0; reads < kMaxReadsPerCycle && !client.closing; ++reads) {
    const ssize_t received = ::recv(client.socket.get(), chunk.data(), chunk.size(), 0);
    if (received == 0) {
      client.closing = true;
      client.close_reason = "connection closed by the robot";
      return;
    }
    if (received < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        client.closing = true;
        client.close_reason = "connection error: " + errno_to_string(errno);
      }
      return;
    }

    client.last_activity = Clock::now();
    client.decoder.feed(chunk.data(), static_cast<std::size_t>(received));

    Frame frame;
    for (;; ) {
      const DecodeStatus status = client.decoder.next(frame);
      if (status == DecodeStatus::kNeedMoreData) {
        break;
      }
      if (status == DecodeStatus::kError) {
        // A corrupt stream cannot be resynchronized safely. Closing forces the
        // robot to reconnect and start again from a clean frame boundary.
        client.closing = true;
        client.close_reason = "protocol error: " + client.decoder.error();
        return;
      }
      if (frame.type == FrameType::kHello) {
        client.info.robot_id = sanitize_robot_id(frame.payload);
      }
      if (handlers_.on_frame) {
        handlers_.on_frame(client.info, frame);
      }
    }
  }
}

void FrameServer::drop_closed_clients()
{
  for (auto client = clients_.begin(); client != clients_.end(); ) {
    if (client->closing) {
      if (handlers_.on_disconnect) {
        handlers_.on_disconnect(client->info, client->close_reason);
      }
      client = clients_.erase(client);
    } else {
      ++client;
    }
  }
}

}  // namespace robot_server_comm
