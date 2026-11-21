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

#include "robot_server_comm/core/config_utils.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_server_comm
{

bool is_valid_port(std::int64_t port)
{
  return port >= 1 && port <= 65535;
}

std::vector<std::string> clean_string_list(const std::vector<std::string> & entries)
{
  std::vector<std::string> cleaned;
  for (const std::string & entry : entries) {
    if (!entry.empty() && std::find(cleaned.begin(), cleaned.end(), entry) == cleaned.end()) {
      cleaned.push_back(entry);
    }
  }
  return cleaned;
}

std::map<std::string, double> parse_rate_limits(const std::vector<std::string> & entries)
{
  std::map<std::string, double> limits;
  for (const std::string & entry : entries) {
    if (entry.empty()) {
      continue;
    }
    // The separator is the LAST colon, topic names themselves never contain one.
    const std::size_t separator = entry.rfind(':');
    if (separator == std::string::npos || separator == 0 || separator + 1 >= entry.size()) {
      throw std::invalid_argument(
              "rate limit '" + entry + "' is not written as 'topic:max_hz'");
    }
    const std::string topic = entry.substr(0, separator);
    const std::string rate_text = entry.substr(separator + 1);

    double rate = 0.0;
    std::size_t parsed = 0;
    try {
      rate = std::stod(rate_text, &parsed);
    } catch (const std::exception &) {
      parsed = 0;
    }
    if (parsed != rate_text.size() || !std::isfinite(rate) || rate <= 0.0) {
      throw std::invalid_argument(
              "rate limit '" + entry + "' needs a positive number of hertz after the colon");
    }
    limits[topic] = rate;
  }
  return limits;
}

RateLimiter::RateLimiter(double max_hz)
{
  if (std::isfinite(max_hz) && max_hz > 0.0) {
    min_period_ = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / max_hz));
  }
}

bool RateLimiter::allow(Clock::time_point now)
{
  if (min_period_ == Clock::duration::zero()) {
    return true;
  }
  if (!has_passed_) {
    has_passed_ = true;
    last_pass_ = now;
    return true;
  }
  const Clock::duration elapsed = now - last_pass_;
  if (elapsed < min_period_) {
    return false;
  }
  // The reference moves by exactly one period when the source is regular, so
  // timing jitter of the source does not lower the average output rate. After
  // a long pause the reference restarts from the current time.
  last_pass_ = elapsed >= 2 * min_period_ ? now : last_pass_ + min_period_;
  return true;
}

}  // namespace robot_server_comm
