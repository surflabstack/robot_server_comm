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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "robot_server_comm/protocol/crc32.hpp"
#include "robot_server_comm/transport/tcp_transport.hpp"
#include "test_helpers.hpp"

namespace robot_server_comm
{

using std::chrono::milliseconds;
using test_support::LoopbackListener;
using test_support::read_exact;
using test_support::wait_until;

namespace
{

TcpTransportConfig config_for(std::uint16_t port)
{
  TcpTransportConfig config;
  config.host = "127.0.0.1";
  config.port = port;
  config.connect_timeout = milliseconds(1000);
  config.send_timeout = milliseconds(300);
  return config;
}

std::vector<std::uint8_t> pattern(std::size_t size, std::uint8_t seed)
{
  std::vector<std::uint8_t> data(size);
  for (std::size_t i = 0; i < size; ++i) {
    data[i] = static_cast<std::uint8_t>(seed + i * 31);
  }
  return data;
}

}  // namespace

TEST(TcpTransport, ConnectFailsCleanlyWhenNobodyListens)
{
  std::uint16_t free_port = 0;
  {
    LoopbackListener listener;
    free_port = listener.port();
  }
  TcpTransport transport(config_for(free_port));
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(transport.connect());
  EXPECT_LT(std::chrono::steady_clock::now() - start, milliseconds(1500));
  EXPECT_FALSE(transport.is_connected());
  EXPECT_FALSE(transport.last_error().empty());
  EXPECT_FALSE(transport.send({{nullptr, 0}}));
}

TEST(TcpTransport, SendsSeveralBuffersAsOneOrderedMessage)
{
  LoopbackListener listener;
  TcpTransport transport(config_for(listener.port()));
  ASSERT_TRUE(transport.connect()) << transport.last_error();
  UniqueFd peer = listener.accept_client();
  ASSERT_TRUE(peer.valid());
  EXPECT_TRUE(transport.is_connected());
  EXPECT_EQ("tcp://127.0.0.1:" + std::to_string(listener.port()), transport.description());

  const std::vector<std::uint8_t> first = pattern(40, 1);
  const std::vector<std::uint8_t> second = pattern(1000, 2);
  ASSERT_TRUE(
    transport.send(
      {{first.data(), first.size()}, {nullptr, 0}, {second.data(), second.size()}}));

  std::vector<std::uint8_t> received(first.size() + second.size());
  ASSERT_TRUE(read_exact(peer.get(), received.data(), received.size()));
  std::vector<std::uint8_t> expected = first;
  expected.insert(expected.end(), second.begin(), second.end());
  EXPECT_EQ(expected, received);
}

TEST(TcpTransport, SendsAMessageMuchLargerThanTheSocketBuffer)
{
  LoopbackListener listener;
  TcpTransportConfig config = config_for(listener.port());
  config.send_timeout = milliseconds(5000);
  TcpTransport transport(config);
  ASSERT_TRUE(transport.connect()) << transport.last_error();
  UniqueFd peer = listener.accept_client();
  ASSERT_TRUE(peer.valid());

  const std::vector<std::uint8_t> header = pattern(64, 9);
  const std::vector<std::uint8_t> payload = pattern(16u * 1024u * 1024u, 5);
  std::vector<std::uint8_t> received(header.size() + payload.size());
  bool read_ok = false;
  std::thread reader([&peer, &received, &read_ok] {
      read_ok = read_exact(peer.get(), received.data(), received.size(), milliseconds(30000));
    });

  EXPECT_TRUE(transport.send({{header.data(), header.size()}, {payload.data(), payload.size()}}));
  reader.join();
  ASSERT_TRUE(read_ok);
  EXPECT_EQ(
    crc32(payload.data(), payload.size(), crc32(header.data(), header.size())),
    crc32(received.data(), received.size()));
}

TEST(TcpTransport, NoticesWhenTheServerClosesTheConnection)
{
  LoopbackListener listener;
  TcpTransport transport(config_for(listener.port()));
  ASSERT_TRUE(transport.connect());
  UniqueFd peer = listener.accept_client();
  ASSERT_TRUE(peer.valid());
  ASSERT_TRUE(transport.is_connected());

  peer.reset();
  EXPECT_TRUE(wait_until([&transport] {return !transport.is_connected();}));
  EXPECT_NE(std::string::npos, transport.last_error().find("closed"));
}

TEST(TcpTransport, SendGivesUpWhenTheServerStopsReading)
{
  LoopbackListener listener;
  TcpTransport transport(config_for(listener.port()));
  ASSERT_TRUE(transport.connect());
  UniqueFd peer = listener.accept_client();  // accepted, but never read from
  ASSERT_TRUE(peer.valid());

  // Far more than the kernel buffers can absorb: the send must stall, then time out.
  const std::vector<std::uint8_t> payload = pattern(64u * 1024u * 1024u, 3);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(transport.send({{payload.data(), payload.size()}}));
  EXPECT_LT(std::chrono::steady_clock::now() - start, milliseconds(5000));
  EXPECT_NE(std::string::npos, transport.last_error().find("timed out"));
  // A failed send closes the link, no half sent message stays on an open connection.
  EXPECT_FALSE(transport.is_connected());
}

TEST(TcpTransport, CanReconnectAfterALostConnection)
{
  LoopbackListener listener;
  TcpTransport transport(config_for(listener.port()));
  ASSERT_TRUE(transport.connect());
  UniqueFd first_peer = listener.accept_client();
  ASSERT_TRUE(first_peer.valid());
  first_peer.reset();
  ASSERT_TRUE(wait_until([&transport] {return !transport.is_connected();}));

  ASSERT_TRUE(transport.connect()) << transport.last_error();
  UniqueFd second_peer = listener.accept_client();
  ASSERT_TRUE(second_peer.valid());

  const std::vector<std::uint8_t> data = pattern(128, 4);
  ASSERT_TRUE(transport.send({{data.data(), data.size()}}));
  std::vector<std::uint8_t> received(data.size());
  ASSERT_TRUE(read_exact(second_peer.get(), received.data(), received.size()));
  EXPECT_EQ(data, received);
}

}  // namespace robot_server_comm
