#!/bin/bash
set -eo pipefail

# Parse CAN interface arguments
# Default values are set here
CAN_INTERFACE="socketcan"
CAN_CHANNEL="can0"
CAN_BITRATE="125000"
ROS_LOG_LEVEL="info"

while [[ $# -gt 0 ]]; do
    case "$1" in
    --interface)
        CAN_INTERFACE="$2"
        shift 2
        ;;
    --channel)
        CAN_CHANNEL="$2"
        shift 2
        ;;
    --bitrate)
        CAN_BITRATE="$2"
        shift 2
        ;;
    --log-level)
        ROS_LOG_LEVEL="$2"
        shift 2
        ;;
    *)
        break
        ;;
    esac
done

# setup gateway env
# shellcheck disable=SC1091
source "./install/setup.bash"

ros2 launch gateway gateway.launch.py \
    can_interface:="${CAN_INTERFACE}" \
    can_channel:="${CAN_CHANNEL}" \
    can_bitrate:="${CAN_BITRATE}" \
    log_level:="${ROS_LOG_LEVEL}"
