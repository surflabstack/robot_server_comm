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

#include "robot_server_comm/transport/transport_factory.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robot_server_comm/transport/tcp_transport.hpp"

namespace robot_server_comm
{

TransportFactory::TransportFactory()
{
  register_transport(
    "tcp", [](const TransportSettings & settings) -> std::unique_ptr<Transport> {
      TcpTransportConfig config;
      config.host = settings.host;
      config.port = settings.port;
      config.connect_timeout = settings.connect_timeout;
      config.send_timeout = settings.send_timeout;
      return std::make_unique<TcpTransport>(config);
    });
}

TransportFactory & TransportFactory::instance()
{
  static TransportFactory factory;
  return factory;
}

void TransportFactory::register_transport(const std::string & protocol, Creator creator)
{
  creators_[protocol] = std::move(creator);
}

std::unique_ptr<Transport> TransportFactory::create(
  const std::string & protocol, const TransportSettings & settings) const
{
  const auto found = creators_.find(protocol);
  if (found == creators_.end()) {
    return nullptr;
  }
  return found->second(settings);
}

std::vector<std::string> TransportFactory::available_protocols() const
{
  std::vector<std::string> names;
  names.reserve(creators_.size());
  for (const auto & entry : creators_) {
    names.push_back(entry.first);
  }
  return names;
}

}  // namespace robot_server_comm
