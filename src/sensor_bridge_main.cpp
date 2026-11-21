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

#include <cstdlib>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot_server_comm/sensor_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = EXIT_SUCCESS;
  try {
    auto node = std::make_shared<robot_server_comm::SensorBridgeNode>();
    // Several threads: every topic has its own callback group, so a large
    // message being copied (image, point cloud) does not delay the other topics.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    // Typically an invalid parameter: say why and stop instead of running misconfigured.
    RCLCPP_FATAL(rclcpp::get_logger("sensor_bridge"), "%s", error.what());
    exit_code = EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return exit_code;
}
