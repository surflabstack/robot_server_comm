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

#ifndef ROBOT_SERVER_COMM__CORE__CONFIG_UTILS_HPP_
#define ROBOT_SERVER_COMM__CORE__CONFIG_UTILS_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace robot_server_comm
{

/// Returns true when the value is a usable TCP or UDP port (1 to 65535).
bool is_valid_port(std::int64_t port);

/// Removes empty entries and duplicates, keeping the original order.
std::vector<std::string> clean_string_list(const std::vector<std::string> & entries);

/// Parses per topic rate limits written as "topic:max_hz", for example
/// "/camera/image_color:5.0". Empty entries are ignored.
///
/// \throws std::invalid_argument with a message naming the faulty entry
std::map<std::string, double> parse_rate_limits(const std::vector<std::string> & entries);

/// Lets at most `max_hz` events per second through. A limit of 0 lets everything through.
class RateLimiter
{
public:
  using Clock = std::chrono::steady_clock;

  explicit RateLimiter(double max_hz = 0.0);

  /// Returns true when an event happening at `now` may pass.
  bool allow(Clock::time_point now = Clock::now());

private:
  Clock::duration min_period_{Clock::duration::zero()};
  Clock::time_point last_pass_{};
  bool has_passed_{false};
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__CORE__CONFIG_UTILS_HPP_
