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

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_server_comm/core/config_utils.hpp"
#include "robot_server_comm/transport/transport_factory.hpp"

namespace robot_server_comm
{

using std::chrono::milliseconds;

TEST(ConfigUtils, ValidatesPorts)
{
  EXPECT_TRUE(is_valid_port(1));
  EXPECT_TRUE(is_valid_port(9000));
  EXPECT_TRUE(is_valid_port(65535));
  EXPECT_FALSE(is_valid_port(0));
  EXPECT_FALSE(is_valid_port(-1));
  EXPECT_FALSE(is_valid_port(65536));
}

TEST(ConfigUtils, CleansStringLists)
{
  const std::vector<std::string> cleaned = clean_string_list({"", "/a", "/b", "/a", ""});
  ASSERT_EQ(2u, cleaned.size());
  EXPECT_EQ("/a", cleaned[0]);
  EXPECT_EQ("/b", cleaned[1]);
  EXPECT_TRUE(clean_string_list({""}).empty());
}

TEST(ConfigUtils, ParsesRateLimits)
{
  const auto limits = parse_rate_limits({"/camera/image_color:5.0", "", "/velodyne_points:2"});
  ASSERT_EQ(2u, limits.size());
  EXPECT_DOUBLE_EQ(5.0, limits.at("/camera/image_color"));
  EXPECT_DOUBLE_EQ(2.0, limits.at("/velodyne_points"));
  EXPECT_TRUE(parse_rate_limits({""}).empty());
}

TEST(ConfigUtils, RejectsMalformedRateLimits)
{
  EXPECT_THROW(parse_rate_limits({"/imu/data"}), std::invalid_argument);
  EXPECT_THROW(parse_rate_limits({"/imu/data:"}), std::invalid_argument);
  EXPECT_THROW(parse_rate_limits({":5"}), std::invalid_argument);
  EXPECT_THROW(parse_rate_limits({"/imu/data:fast"}), std::invalid_argument);
  EXPECT_THROW(parse_rate_limits({"/imu/data:5hz"}), std::invalid_argument);
  EXPECT_THROW(parse_rate_limits({"/imu/data:0"}), std::invalid_argument);
  EXPECT_THROW(parse_rate_limits({"/imu/data:-3"}), std::invalid_argument);
  EXPECT_THROW(parse_rate_limits({"/imu/data:nan"}), std::invalid_argument);
}

TEST(RateLimiter, LetsEverythingThroughWithoutALimit)
{
  RateLimiter limiter;
  const auto start = RateLimiter::Clock::now();
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(limiter.allow(start));
  }
}

TEST(RateLimiter, KeepsTheConfiguredAverageRate)
{
  // A 100 Hz source limited to 10 Hz over 2 simulated seconds must pass about 20 messages.
  RateLimiter limiter(10.0);
  const auto start = RateLimiter::Clock::now();
  int passed = 0;
  for (int i = 0; i < 200; ++i) {
    if (limiter.allow(start + milliseconds(10 * i))) {
      ++passed;
    }
  }
  EXPECT_GE(passed, 19);
  EXPECT_LE(passed, 21);
}

TEST(RateLimiter, SourceJitterDoesNotLowerTheOutputRate)
{
  // A 10 Hz source limited to 5 Hz whose messages arrive 1 ms early or late.
  RateLimiter limiter(5.0);
  const auto start = RateLimiter::Clock::now();
  int passed = 0;
  for (int i = 0; i < 100; ++i) {
    const int jitter_ms = (i % 2 == 0) ? -1 : 1;
    if (limiter.allow(start + milliseconds(100 * i + jitter_ms))) {
      ++passed;
    }
  }
  EXPECT_GE(passed, 48);
  EXPECT_LE(passed, 51);
}

TEST(TransportFactory, CreatesTcpAndRefusesUnknownProtocols)
{
  const TransportFactory & factory = TransportFactory::instance();
  TransportSettings settings;
  settings.host = "127.0.0.1";
  settings.port = 9000;

  const auto tcp = factory.create("tcp", settings);
  ASSERT_NE(nullptr, tcp);
  EXPECT_EQ("tcp://127.0.0.1:9000", tcp->description());
  EXPECT_FALSE(tcp->is_connected());

  EXPECT_EQ(nullptr, factory.create("carrier_pigeon", settings));

  const std::vector<std::string> protocols = factory.available_protocols();
  EXPECT_NE(protocols.end(), std::find(protocols.begin(), protocols.end(), "tcp"));
}

}  // namespace robot_server_comm
