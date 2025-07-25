#!/bin/bash
set -eo pipefail

CAN_INTERFACE="socketcan"
CAN_CHANNEL="can0"
CAN_BITRATE="125000"
ROS_LOG_LEVEL="info"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
    -i | --interface)
        CAN_INTERFACE="$2"
        shift 2
        ;;
    -c | --channel)
        CAN_CHANNEL="$2"
        shift 2
        ;;
    -b | --bitrate)
        CAN_BITRATE="$2"
        shift 2
        ;;
    -l | --log-level)
        ROS_LOG_LEVEL="$2"
        shift 2
        ;;
    -h | --help)
        echo "Usage: $0 [--interface <can_interface>] [--channel <can_channel>] [--bitrate <can_bitrate>] [--log-level <log_level>]"
        echo
        echo "Options:"
        echo "  -i, --interface <can_interface>   CAN interface to use (default: socketcan)"
        echo "  -c, --channel <can_channel>       CAN channel to use (default: can0)"
        echo "  -b, --bitrate <can_bitrate>       CAN bitrate to use (default: 125000)"
        echo "  -l, --log-level <log_level>       ROS2 log level (default: info)"
        echo "  -h, --help                    Show this help message and exit"
        echo
        echo "Supported ROS2 log levels:"
        echo "  UNSET, DEBUG, INFO, WARN, ERROR, FATAL"
        echo
        echo "Examples:"
        echo "  $0 --interface socketcan --channel can0 --bitrate 125000 --log-level info"
        echo "  $0 --log-level warn"
        exit 0
        ;;
    *)
        echo "Unknown argument: $1"
        echo "Usage: $0 [--interface <can_interface>] [--channel <can_channel>] [--bitrate <can_bitrate>] [--log-level <log_level>]"
        echo "Try '$0 --help' for more information."
        exit 1
        ;;
    esac
done

# Set up CAN interface
if [[ ${CAN_INTERFACE} == "socketcan" ]]; then
    echo "Setting up socketcan interface ${CAN_CHANNEL} with bitrate ${CAN_BITRATE}"
    sudo ip link set "${CAN_CHANNEL}" down 2>/dev/null || true
    sudo ip link set "${CAN_CHANNEL}" type can bitrate "${CAN_BITRATE}" restart-ms 100
    sudo ip link set "${CAN_CHANNEL}" up
fi

# Launch container, passing in arguments
docker run --rm -it \
    --name rover-ros-gateway \
    --network host \
    --ipc host \
    --pid host \
    --cap-add=NET_ADMIN \
    rover-ros-gateway \
    --interface "${CAN_INTERFACE}" \
    --channel "${CAN_CHANNEL}" \
    --bitrate "${CAN_BITRATE}" \
    --log-level "${ROS_LOG_LEVEL}"
