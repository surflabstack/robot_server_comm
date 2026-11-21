cmake_minimum_required(VERSION 3.16)
project(robot_server_comm VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# No architecture specific flag (-march, SSE, NEON...) is used anywhere, so the
# same sources build on x86_64 and arm64.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Threads REQUIRED)

# The package builds in two ways:
#   - with colcon inside a ROS 2 workspace: everything (node, server, tests),
#   - with plain CMake on a machine without ROS: the core library, the local
#     server and the unit tests. This is what a server side team would use.
find_package(ament_cmake QUIET)

# ---------------------------------------------------------------------------
# Core library: protocol, transports, transmitter, server. No ROS dependency.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core STATIC
  src/core/config_utils.cpp
  src/core/data_transmitter.cpp
  src/net/socket_utils.cpp
  src/protocol/crc32.cpp
  src/protocol/frame_codec.cpp
  src/server/frame_printer.cpp
  src/server/frame_server.cpp
  src/server/payload_summary.cpp
  src/transport/tcp_transport.cpp
  src/transport/transport_factory.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Threads::Threads)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Local Mission Planner Server used to test the robot side (plain C++, no ROS).
add_executable(mission_planner_server src/server/mission_planner_server_main.cpp)
target_link_libraries(mission_planner_server PRIVATE ${PROJECT_NAME}_core)

set(ROBOT_SERVER_COMM_TESTS
  test_bounded_queue
  test_config_utils
  test_data_transmitter
  test_end_to_end
  test_frame_codec
  test_payload_summary
  test_tcp_transport
)

if(ament_cmake_FOUND)
  # -------------------------------------------------------------------------
  # ROS 2 build
  # -------------------------------------------------------------------------
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)

  # The node is built as a component (shared library). It runs either as the
  # stand-alone executable below or inside a component container, next to
  # other nodes of the robot.
  add_library(sensor_bridge_component SHARED src/sensor_bridge_node.cpp)
  target_link_libraries(sensor_bridge_component
    PUBLIC ${PROJECT_NAME}_core rclcpp::rclcpp
    PRIVATE rclcpp_components::component
  )
  rclcpp_components_register_nodes(sensor_bridge_component
    "robot_server_comm::SensorBridgeNode"
  )

  add_executable(sensor_bridge_node src/sensor_bridge_main.cpp)
  target_link_libraries(sensor_bridge_node PRIVATE sensor_bridge_component)

  install(TARGETS sensor_bridge_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS sensor_bridge_node mission_planner_server
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)

    # Style checks (the linters are the ament_cmake_* test dependencies of package.xml).
    ament_lint_auto_find_test_dependencies()

    # Tests of the ROS independent layers.
    foreach(test_name IN LISTS ROBOT_SERVER_COMM_TESTS)
      ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 120)
      if(TARGET ${test_name})
        target_link_libraries(${test_name} ${PROJECT_NAME}_core)
      endif()
    endforeach()

    # Test of the node itself: real publishers, the node and a server in one process.
    ament_add_gtest(test_sensor_bridge_node test/test_sensor_bridge_node.cpp TIMEOUT 120)
    if(TARGET test_sensor_bridge_node)
      target_link_libraries(test_sensor_bridge_node
        sensor_bridge_component
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
      )
    endif()
  endif()

  ament_package()
else()
  # -------------------------------------------------------------------------
  # Plain CMake build (no ROS): core library, server and tests only
  # -------------------------------------------------------------------------
  message(STATUS "ament_cmake not found: building without the ROS 2 node")
  include(CTest)
  if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    foreach(test_name IN LISTS ROBOT_SERVER_COMM_TESTS)
      add_executable(${test_name} test/${test_name}.cpp)
      target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME}_core GTest::gtest_main)
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  endif()
  install(TARGETS mission_planner_server RUNTIME DESTINATION bin)
endif()
