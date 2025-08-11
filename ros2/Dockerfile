ARG ROS_DISTRO=jazzy

# Build stage
FROM ros:$ROS_DISTRO AS base

WORKDIR /ros2_ws

COPY ros2 .

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" && colcon build'

# Run stage
FROM ros:$ROS_DISTRO-ros-core

RUN apt-get update \
    && apt-get install -y python3-can \
    && rm -rf /var/lib/apt/lists/*

COPY rover_py /opt/rover_py
ENV PYTHONPATH=/opt/rover_py

WORKDIR /ros2_ws
COPY --from=base /ros2_ws .

RUN chmod a+x ./entrypoint.sh

# Create user if it doesn't exist
RUN bash -c 'id -u 1000 &>/dev/null || useradd -m -s /bin/bash ubuntu'

RUN chown -R 1000:1000 /ros2_ws

USER 1000:1000

ENTRYPOINT [ "./entrypoint.sh" ]
