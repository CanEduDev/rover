ARG ROS_DISTRO=jazzy

# Build stage
FROM ros:$ROS_DISTRO AS cacher

# Overwrite defaults to persist minimal cache
RUN rosdep update --rosdistro $ROS_DISTRO && \
    cat <<EOF > /etc/apt/apt.conf.d/docker-clean && apt-get update
APT::Install-Recommends "false";
APT::Install-Suggests "false";
EOF

COPY ros2 /ros2_ws

# Derive build/exec dependencies
RUN bash -eo pipefail <<'EOF'
declare -A types=(
  [exec]="--dependency-types=exec"
  [build]="")
for type in "${!types[@]}"; do
  rosdep install -y \
    --from-paths /ros2_ws \
    --ignore-src \
    --reinstall \
    --simulate \
    ${types[$type]} \
    | grep 'apt-get install' \
    | awk '{gsub(/'\''/,"",$4); print $4}' \
    | sort -u > /tmp/${type}_debs.txt
done
EOF

FROM ros:$ROS_DISTRO AS builder

# Install build dependencies
COPY --from=cacher /tmp/build_debs.txt /tmp/build_debs.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    < /tmp/build_debs.txt xargs apt-get install -y

COPY --from=cacher /ros2_ws /ros2_ws
WORKDIR /ros2_ws

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.sh && colcon build --mixin release"

# Run stage
FROM ros:$ROS_DISTRO-ros-core

# Install exec dependencies
COPY --from=cacher /tmp/exec_debs.txt /tmp/exec_debs.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    < /tmp/exec_debs.txt xargs apt-get install -y

COPY rover_py /opt/rover_py
ENV PYTHONPATH=/opt/rover_py

COPY --from=builder /ros2_ws /ros2_ws
WORKDIR /ros2_ws

RUN chmod a+x ./entrypoint.sh

# Create default user if it doesn't exist
RUN bash -c 'id -u 1000 &>/dev/null || useradd -m -s /bin/bash ubuntu'

RUN chown -R 1000:1000 /ros2_ws

USER 1000:1000

ENTRYPOINT [ "./entrypoint.sh" ]
