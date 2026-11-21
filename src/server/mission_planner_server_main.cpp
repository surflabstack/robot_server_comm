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

// Local stand-in for the Mission Planner Server.
//
// It listens on a TCP port of this machine, receives the frames sent by the
// robot and prints them in the terminal. It has no ROS dependency: it only
// needs the wire protocol, exactly like a real server would.

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "robot_server_comm/server/frame_printer.hpp"
#include "robot_server_comm/server/frame_server.hpp"

namespace
{

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int signal_number)
{
  (void)signal_number;
  g_stop_requested = 1;
}

struct Options
{
  robot_server_comm::FrameServerConfig server;
  robot_server_comm::FramePrinterConfig printer;
  bool show_help{false};
};

void print_usage(const char * program)
{
  std::cout <<
    "Usage: " << program << " [options]\n"
    "\n"
    "Local Mission Planner Server: receives robot sensor frames and prints them.\n"
    "\n"
    "Options:\n"
    "  --host ADDRESS          address to listen on (default 127.0.0.1, local machine only)\n"
    "  --port PORT             TCP port to listen on (default 9000)\n"
    "  --print-period-ms MS    shortest time between two printed messages of one topic,\n"
    "                          0 prints every message (default 500)\n"
    "  --stats-period-s S      period of the statistics table, 0 disables it (default 5)\n"
    "  --client-timeout-ms MS  disconnect a robot that stays silent this long (default 5000)\n"
    "  --max-payload-mb MB     largest accepted message in megabytes (default 64)\n"
    "  -h, --help              show this help\n";
}

// Parses a whole number and checks its range. Throws std::invalid_argument with a clear text.
std::int64_t parse_integer(
  const std::string & option, const std::string & text, std::int64_t minimum,
  std::int64_t maximum)
{
  std::size_t parsed = 0;
  std::int64_t value = 0;
  try {
    value = std::stoll(text, &parsed);
  } catch (const std::exception &) {
    parsed = 0;
  }
  if (parsed == 0 || parsed != text.size() || value < minimum || value > maximum) {
    throw std::invalid_argument(
            option + " expects a whole number between " + std::to_string(minimum) + " and " +
            std::to_string(maximum) + ", got '" + text + "'");
  }
  return value;
}

Options parse_arguments(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "-h" || argument == "--help") {
      options.show_help = true;
      return options;
    }
    if (argument == "--ros-args") {
      // Arguments added by ROS launch tools are not for this program.
      break;
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("option " + argument + " needs a value");
    }
    const std::string value = argv[++i];
    if (argument == "--host") {
      if (value.empty()) {
        throw std::invalid_argument("--host cannot be empty");
      }
      options.server.bind_address = value;
    } else if (argument == "--port") {
      options.server.port = static_cast<std::uint16_t>(parse_integer(argument, value, 1, 65535));
    } else if (argument == "--print-period-ms") {
      options.printer.print_period =
        std::chrono::milliseconds(parse_integer(argument, value, 0, 3600000));
    } else if (argument == "--stats-period-s") {
      options.printer.stats_period =
        std::chrono::seconds(parse_integer(argument, value, 0, 3600));
    } else if (argument == "--client-timeout-ms") {
      options.server.client_timeout =
        std::chrono::milliseconds(parse_integer(argument, value, 100, 3600000));
    } else if (argument == "--max-payload-mb") {
      options.server.max_payload_size =
        static_cast<std::size_t>(parse_integer(argument, value, 1, 1024)) * 1024u * 1024u;
    } else {
      throw std::invalid_argument("unknown option " + argument);
    }
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    options = parse_arguments(argc, argv);
  } catch (const std::invalid_argument & error) {
    std::cerr << "Error: " << error.what() << "\n\n";
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }
  if (options.show_help) {
    print_usage(argv[0]);
    return EXIT_SUCCESS;
  }

  // Print line by line even when the output is a pipe (launch files, docker logs).
  std::cout.setf(std::ios::unitbuf);

  using robot_server_comm::ClientInfo;
  using robot_server_comm::Frame;

  robot_server_comm::FramePrinter printer(std::cout, options.printer);
  robot_server_comm::FrameServerHandlers handlers;
  handlers.on_connect = [&printer](const ClientInfo & client) {
      printer.on_connect(client);
    };
  handlers.on_frame = [&printer](const ClientInfo & client, const Frame & frame) {
      printer.on_frame(client, frame);
    };
  handlers.on_disconnect = [&printer](const ClientInfo & client, const std::string & reason) {
      printer.on_disconnect(client, reason);
    };

  robot_server_comm::FrameServer server(options.server, handlers);
  std::string error;
  if (!server.start(error)) {
    std::cerr << "Error: cannot start the server: " << error << std::endl;
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::cout << "[SERVER] Mission Planner Server (local test server) listening on " <<
    options.server.bind_address << ":" << server.port() << ", waiting for a robot..." <<
    std::endl;

  while (g_stop_requested == 0) {
    server.poll_once(std::chrono::milliseconds(200));
    printer.tick();
  }

  printer.print_stats();
  const robot_server_comm::ReceptionTotals & totals = printer.totals();
  std::cout << "[SERVER] stopped. Received " << totals.data_frames << " messages (" <<
    robot_server_comm::format_size(totals.data_bytes) << "), " << totals.lost_messages <<
    " missing (gaps in the sequence numbers)." << std::endl;
  server.stop();
  return EXIT_SUCCESS;
}
