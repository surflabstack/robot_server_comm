# Copyright 2026 Anas Derkaoui
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch the robot to server communication demo.

Starts, depending on the arguments:
  - the local Mission Planner Server (launch_server:=true, default),
  - the sensor bridge node that forwards ROS 2 topics to the server,
  - the replay of a rosbag (bag:=/path/to/bag), optional.

Typical commands:
  ros2 launch robot_server_comm robot_server_comm.launch.py
  ros2 launch robot_server_comm robot_server_comm.launch.py launch_server:=false
  ros2 launch robot_server_comm robot_server_comm.launch.py bag:=/data/HK-Data20190117
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

PACKAGE = 'robot_server_comm'


def generate_launch_description():
    server_host = LaunchConfiguration('server_host')
    server_port = LaunchConfiguration('server_port')
    robot_id = LaunchConfiguration('robot_id')
    params_file = LaunchConfiguration('params_file')
    launch_server = LaunchConfiguration('launch_server')
    print_period_ms = LaunchConfiguration('print_period_ms')
    bag = LaunchConfiguration('bag')
    bag_rate = LaunchConfiguration('bag_rate')
    log_level = LaunchConfiguration('log_level')

    arguments = [
        DeclareLaunchArgument(
            'server_host', default_value='127.0.0.1',
            description='Address of the Mission Planner Server.'),
        DeclareLaunchArgument(
            'server_port', default_value='9000',
            description='TCP port of the Mission Planner Server.'),
        DeclareLaunchArgument(
            'robot_id', default_value='urbanloco_vehicle',
            description='Name the robot announces to the server.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare(PACKAGE), 'config', 'sensor_bridge.yaml']),
            description='YAML file with the parameters of the sensor bridge node.'),
        DeclareLaunchArgument(
            'launch_server', default_value='true',
            description='Start the local Mission Planner Server in this launch. Set to false '
                        'to run the server in its own terminal.'),
        DeclareLaunchArgument(
            'print_period_ms', default_value='500',
            description='Server side: shortest time between two printed messages of one '
                        'topic, 0 prints every message.'),
        DeclareLaunchArgument(
            'bag', default_value='',
            description='Path of a rosbag to replay. Empty: no replay, play it yourself with '
                        '"ros2 bag play".'),
        DeclareLaunchArgument(
            'bag_rate', default_value='1.0',
            description='Replay speed of the rosbag.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Log level of the sensor bridge node.'),
    ]

    # The server is a plain C++ program (no ROS), so it is started as a process.
    server = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix(PACKAGE), 'lib', PACKAGE, 'mission_planner_server']),
            '--host', server_host,
            '--port', server_port,
            '--print-period-ms', print_period_ms,
        ],
        name='mission_planner_server',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(launch_server),
    )

    bridge = Node(
        package=PACKAGE,
        executable='sensor_bridge_node',
        name='sensor_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'server_host': ParameterValue(server_host, value_type=str),
                'server_port': ParameterValue(server_port, value_type=int),
                'robot_id': ParameterValue(robot_id, value_type=str),
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Optional replay, started once the server and the bridge are up. The player
    # advertises its topics at once and waits one second ("--delay") before the
    # first message, which gives the bridge the time to subscribe to all of them.
    bag_player = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag, '--rate', bag_rate, '--delay', '1.0'],
                name='rosbag_player',
                output='screen',
            ),
        ],
        condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
    )

    return LaunchDescription(arguments + [server, bridge, bag_player])
