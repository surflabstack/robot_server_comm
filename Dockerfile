# Image that builds, tests and runs the robot_server_comm package.
#
# The official ROS images exist for amd64 and arm64, so the same Dockerfile
# works on a PC and on an ARM robot computer:
#   docker build -t robot_server_comm .
#   docker buildx build --platform linux/amd64,linux/arm64 -t robot_server_comm .
#
# Another ROS 2 distribution can be selected at build time:
#   docker build --build-arg ROS_DISTRO=humble -t robot_server_comm .
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Set to false to skip the unit tests during the image build (for example when
# cross-building for another CPU architecture under emulation, where tests are slow).
ARG RUN_TESTS=true

# Optional limit of parallel compile jobs, for computers with little memory:
#   docker build --build-arg MAKEFLAGS=-j1 -t robot_server_comm .
# Empty by default: colcon then uses every CPU core.
ARG MAKEFLAGS

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /ws

# 1. Dependencies. Only package.xml is copied first, so this layer is rebuilt
#    only when the dependency list changes, not at every source change.
COPY package.xml src/robot_server_comm/package.xml
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
    && rm -rf /var/lib/apt/lists/*

# 2. Message packages of the dataset. HK-Data20190117 also has NovAtel and u-blox
#    topics; "ros2 bag play" only plays a topic whose message package is installed.
#    With these, 19 of the 20 topics play instead of the 6 standard ones (the 20th,
#    INSCOV, no longer exists in the current NovAtel package).
#    The package itself does not need them. Leave them out with:
#      docker build --build-arg DATASET_MSG_PACKAGES= -t robot_server_comm .
ARG DATASET_MSG_PACKAGES="ublox-msgs novatel-oem7-msgs"
RUN if [ -n "${DATASET_MSG_PACKAGES}" ]; then \
      apt-get update \
      && apt-get install -y --no-install-recommends \
         $(for p in ${DATASET_MSG_PACKAGES}; do echo "ros-${ROS_DISTRO}-${p}"; done) \
      && rm -rf /var/lib/apt/lists/*; \
    fi

# 3. Sources.
COPY . src/robot_server_comm

# 4. Build, then run the tests so a broken image can never be produced silently.
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && if [ "${RUN_TESTS}" = "true" ]; then \
         colcon test --event-handlers console_cohesion+ \
         && colcon test-result --verbose; \
       fi

# 5. Run time environment: every command started in the container sees ROS 2
#    and the workspace.
RUN cp src/robot_server_comm/docker/entrypoint.sh /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command: local server + sensor bridge. Rosbags are expected in /data.
CMD ["ros2", "launch", "robot_server_comm", "robot_server_comm.launch.py"]
