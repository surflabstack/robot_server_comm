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

#ifndef ROBOT_SERVER_COMM__TRANSPORT__TRANSPORT_HPP_
#define ROBOT_SERVER_COMM__TRANSPORT__TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_server_comm
{

/// Read only view on a block of bytes owned by somebody else.
struct ConstBuffer
{
  const std::uint8_t * data{nullptr};
  std::size_t size{0};
};

/// Abstract link between the robot and the server.
///
/// The transmitter only talks to this interface, so supporting another protocol
/// (UDP, MQTT, WebSocket, a serial radio...) means writing one new class that
/// implements it and registering it in the transport factory. Nothing else changes.
///
/// A transport is used by a single thread (the transmitter worker). Every call
/// must return within a bounded time so the worker can be stopped cleanly.
class Transport
{
public:
  virtual ~Transport() = default;

  /// Tries to open the link. Returns true when the link is ready to send.
  virtual bool connect() = 0;

  /// Closes the link. Safe to call when already closed.
  virtual void disconnect() = 0;

  /// Returns true while the link is believed to be up.
  virtual bool is_connected() = 0;

  /// Sends one message made of several buffers, in order, as a single unit.
  /// Either every byte is handed to the link or the call fails and the link
  /// is closed (no half sent message is left on an open link).
  virtual bool send(const std::vector<ConstBuffer> & buffers) = 0;

  /// Text describing the last failure, for logs.
  virtual std::string last_error() const = 0;

  /// Text describing the remote end, for example "tcp://127.0.0.1:9000".
  virtual std::string description() const = 0;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__TRANSPORT__TRANSPORT_HPP_
