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

#include "robot_server_comm/transport/tcp_transport.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// MSG_NOSIGNAL does not exist on every POSIX system (macOS uses SO_NOSIGPIPE).
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace robot_server_comm
{

namespace
{

using Clock = std::chrono::steady_clock;

// Largest number of buffers passed to one sendmsg() call.
constexpr std::size_t kMaxIovPerCall = 16;

// poll() on one descriptor, restarted when a signal interrupts it.
// Returns >0 when the event fired, 0 on timeout, <0 on error.
int poll_one(int fd, short events, std::chrono::milliseconds timeout)  // NOLINT(runtime/int)
{
  const auto deadline = Clock::now() + timeout;
  for (;; ) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - Clock::now());
    const int wait_ms = static_cast<int>(std::max<std::int64_t>(0, remaining.count()));
    pollfd descriptor{fd, events, 0};
    const int rc = ::poll(&descriptor, 1, wait_ms);
    if (rc < 0 && errno == EINTR) {
      continue;
    }
    return rc;
  }
}

template<typename ValueT>
void set_option(int fd, int level, int name, ValueT value)
{
  // Socket tuning is best effort: a refused option must not prevent the link from working.
  (void)::setsockopt(fd, level, name, &value, sizeof(value));
}

}  // namespace

TcpTransport::TcpTransport(TcpTransportConfig config)
: config_(std::move(config))
{
}

TcpTransport::~TcpTransport()
{
  disconnect();
}

std::string TcpTransport::description() const
{
  return "tcp://" + config_.host + ":" + std::to_string(config_.port);
}

bool TcpTransport::connect()
{
  disconnect();

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_NUMERICSERV;

  addrinfo * raw_results = nullptr;
  const std::string service = std::to_string(config_.port);
  const int rc = ::getaddrinfo(config_.host.c_str(), service.c_str(), &hints, &raw_results);
  if (rc != 0) {
    return fail("cannot resolve host '" + config_.host + "': " + ::gai_strerror(rc));
  }
  const AddrInfoPtr results(raw_results);

  last_error_.clear();
  for (const addrinfo * candidate = results.get(); candidate != nullptr;
    candidate = candidate->ai_next)
  {
    if (connect_to(candidate->ai_addr, candidate->ai_addrlen)) {
      last_error_.clear();
      return true;
    }
  }
  if (last_error_.empty()) {
    last_error_ = "no usable address for host '" + config_.host + "'";
  }
  return false;
}

bool TcpTransport::connect_to(const sockaddr * address, socklen_t length)
{
  UniqueFd fd(::socket(address->sa_family, SOCK_STREAM, 0));
  if (!fd.valid()) {
    return fail("socket() failed: " + errno_to_string(errno));
  }
  if (!set_non_blocking(fd.get())) {
    return fail("cannot switch the socket to non-blocking mode: " + errno_to_string(errno));
  }

  if (::connect(fd.get(), address, length) != 0) {
    if (errno != EINPROGRESS) {
      return fail("connect to " + description() + " failed: " + errno_to_string(errno));
    }
    const int ready = poll_one(fd.get(), POLLOUT, config_.connect_timeout);
    if (ready == 0) {
      return fail("connect to " + description() + " timed out");
    }
    if (ready < 0) {
      return fail("poll() failed while connecting: " + errno_to_string(errno));
    }
    int socket_error = 0;
    socklen_t error_length = sizeof(socket_error);
    if (::getsockopt(fd.get(), SOL_SOCKET, SO_ERROR, &socket_error, &error_length) != 0) {
      return fail("getsockopt() failed while connecting: " + errno_to_string(errno));
    }
    if (socket_error != 0) {
      return fail("connect to " + description() + " failed: " + errno_to_string(socket_error));
    }
  }

  configure_socket(fd.get());
  socket_ = std::move(fd);
  return true;
}

void TcpTransport::configure_socket(int fd) const
{
  if (config_.tcp_no_delay) {
    set_option<int>(fd, IPPROTO_TCP, TCP_NODELAY, 1);
  }
  if (config_.keep_alive) {
    set_option<int>(fd, SOL_SOCKET, SO_KEEPALIVE, 1);
#ifdef TCP_KEEPIDLE
    set_option<int>(fd, IPPROTO_TCP, TCP_KEEPIDLE, 5);
    set_option<int>(fd, IPPROTO_TCP, TCP_KEEPINTVL, 2);
    set_option<int>(fd, IPPROTO_TCP, TCP_KEEPCNT, 3);
#endif
  }
#ifdef TCP_USER_TIMEOUT
  // Give up on data that stays unacknowledged for too long (unplugged cable, lost Wi-Fi)
  // instead of waiting for the default TCP retransmission limit, which is many minutes.
  const auto user_timeout = std::max(config_.connect_timeout, config_.send_timeout) * 3;
  set_option<unsigned int>(
    fd, IPPROTO_TCP, TCP_USER_TIMEOUT, static_cast<unsigned int>(user_timeout.count()));
#endif
#ifdef SO_NOSIGPIPE
  set_option<int>(fd, SOL_SOCKET, SO_NOSIGPIPE, 1);
#endif
}

void TcpTransport::disconnect()
{
  socket_.reset();
}

bool TcpTransport::is_connected()
{
  if (!socket_.valid()) {
    return false;
  }

  // Version 1 of the protocol is one way: the server never sends application
  // data. Reading without blocking is still useful because it is how a closed
  // or reset connection is noticed before the next send. Anything received is
  // discarded, with a bounded number of reads per call.
  std::uint8_t scratch[256];
  for (int attempt = 0; attempt < 16; ++attempt) {
    const ssize_t received = ::recv(socket_.get(), scratch, sizeof(scratch), MSG_DONTWAIT);
    if (received > 0) {
      continue;
    }
    if (received == 0) {
      disconnect();
      fail("connection closed by the server");
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return true;
    }
    const int error_number = errno;
    disconnect();
    fail("connection lost: " + errno_to_string(error_number));
    return false;
  }
  return true;
}

bool TcpTransport::send(const std::vector<ConstBuffer> & buffers)
{
  if (!socket_.valid()) {
    return fail("not connected");
  }

  std::vector<iovec> parts;
  parts.reserve(buffers.size());
  for (const ConstBuffer & buffer : buffers) {
    if (buffer.size > 0 && buffer.data != nullptr) {
      iovec part{};
      part.iov_base = const_cast<std::uint8_t *>(buffer.data);
      part.iov_len = buffer.size;
      parts.push_back(part);
    }
  }

  // sendmsg() may accept only part of the data (full socket buffer), so the
  // loop continues from the first byte that was not taken yet.
  std::size_t index = 0;
  while (index < parts.size()) {
    msghdr message{};
    message.msg_iov = &parts[index];
    message.msg_iovlen = std::min(parts.size() - index, kMaxIovPerCall);

    const ssize_t sent = ::sendmsg(socket_.get(), &message, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (!wait_writable(config_.send_timeout)) {
          disconnect();
          return fail("send timed out, the server is not reading fast enough or the link is down");
        }
        continue;
      }
      const int error_number = errno;
      disconnect();
      return fail("send failed: " + errno_to_string(error_number));
    }

    std::size_t remaining = static_cast<std::size_t>(sent);
    while (index < parts.size() && remaining >= parts[index].iov_len) {
      remaining -= parts[index].iov_len;
      ++index;
    }
    if (index < parts.size() && remaining > 0) {
      parts[index].iov_base = static_cast<std::uint8_t *>(parts[index].iov_base) + remaining;
      parts[index].iov_len -= remaining;
    }
  }
  return true;
}

bool TcpTransport::wait_writable(std::chrono::milliseconds timeout)
{
  return poll_one(socket_.get(), POLLOUT, timeout) > 0;
}

bool TcpTransport::fail(const std::string & reason)
{
  last_error_ = reason;
  return false;
}

}  // namespace robot_server_comm
