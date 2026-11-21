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

#ifndef ROBOT_SERVER_COMM__TRANSPORT__TRANSPORT_FACTORY_HPP_
#define ROBOT_SERVER_COMM__TRANSPORT__TRANSPORT_FACTORY_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "robot_server_comm/transport/transport.hpp"

namespace robot_server_comm
{

/// Settings shared by every transport. A transport reads what it needs.
struct TransportSettings
{
  std::string host{"127.0.0.1"};
  std::uint16_t port{9000};
  std::chrono::milliseconds connect_timeout{2000};
  std::chrono::milliseconds send_timeout{2000};
};

/// Creates transports from a protocol name such as "tcp".
///
/// Adding a protocol does not touch the node or the transmitter:
///   1. write a class that implements Transport,
///   2. register it here with register_transport("name", creator),
///   3. select it with the `protocol` parameter.
class TransportFactory
{
public:
  using Creator = std::function<std::unique_ptr<Transport>(const TransportSettings &)>;

  /// Factory with the transports shipped in this package already registered.
  static TransportFactory & instance();

  /// Registers (or replaces) the creator of a protocol.
  void register_transport(const std::string & protocol, Creator creator);

  /// Creates a transport. Returns nullptr when the protocol is unknown.
  std::unique_ptr<Transport> create(
    const std::string & protocol, const TransportSettings & settings) const;

  /// Names of the registered protocols, sorted.
  std::vector<std::string> available_protocols() const;

private:
  TransportFactory();

  std::map<std::string, Creator> creators_;
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__TRANSPORT__TRANSPORT_FACTORY_HPP_
