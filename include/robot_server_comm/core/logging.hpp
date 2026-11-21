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

#ifndef ROBOT_SERVER_COMM__CORE__LOGGING_HPP_
#define ROBOT_SERVER_COMM__CORE__LOGGING_HPP_

#include <functional>
#include <string>

namespace robot_server_comm
{

/// Severity of a log line produced by the communication core.
enum class LogLevel
{
  kDebug,
  kInfo,
  kWarn,
  kError,
};

/// Sink for log lines. The core library does not depend on ROS, so the node
/// injects a callback that forwards to the ROS logger. It may be called from
/// the transmitter worker thread.
using LogCallback = std::function<void (LogLevel, const std::string &)>;

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__CORE__LOGGING_HPP_
