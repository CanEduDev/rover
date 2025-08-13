#!/bin/bash
set -eo pipefail

# Default values
CAN_INTERFACE="socketcan"
CAN_CHANNEL="can0"
CAN_BITRATE="125000"
ROS_LOG_LEVEL="info"
ROS_DISTRO="jazzy" # Default ROS distribution
DAEMON_ARG=""

usage() {
    SCRIPT_NAME=$(basename "$0")
    cat <<EOF

${SCRIPT_NAME} [-h | --help] [--interface INTERFACE] [--channel CHANNEL] [--bitrate BITRATE] [--log-level LEVEL] [--ros-distro DISTRO] [--daemon]

Run the ros-gateway docker container. Requires docker to be installed
and usable without sudo.

The container connects to a CAN interface and provides ROS2 nodes
for communicating with rover hardware.

args:
    -h, --help                  show this help
    -i, --interface INTERFACE   CAN interface to use (default: socketcan)
    -c, --channel CHANNEL       CAN channel to use (default: can0)
    -b, --bitrate BITRATE       CAN bitrate to use (default: 125000)
    -l, --log-level LEVEL       ROS2 log level (default: info)
    -r, --ros-distro DISTRO     ROS distribution to use (default: jazzy)
    -d, --daemon                run container as daemon in background

Supported ROS2 log levels:
    UNSET, DEBUG, INFO, WARN, ERROR, FATAL

Examples:
    # Run with default settings
    ${SCRIPT_NAME}

    # Run with custom log level and ROS distro
    ${SCRIPT_NAME} --log-level warn --ros-distro humble

    # Run as daemon in background
    ${SCRIPT_NAME} --daemon

    # Run with custom CAN settings
    ${SCRIPT_NAME} --interface socketcan --channel can0 --bitrate 125000
EOF
}

# Function to parse command line arguments
parse_arguments() {
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
        -r | --ros-distro)
            ROS_DISTRO="$2"
            shift 2
            ;;
        -d | --daemon)
            DAEMON_ARG="-d"
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
        esac
    done
}

# Function to set up CAN interface
setup_can_interface() {
    if [[ ${CAN_INTERFACE} == "socketcan" ]]; then
        echo "Setting up socketcan interface ${CAN_CHANNEL} with bitrate ${CAN_BITRATE}"
        sudo ip link set "${CAN_CHANNEL}" down 2>/dev/null || true
        sudo ip link set "${CAN_CHANNEL}" type can bitrate "${CAN_BITRATE}" restart-ms 100
        sudo ip link set "${CAN_CHANNEL}" up
    fi
}

# Function to run container
run_container() {
    echo "Stopping container rover-ros-gateway..."
    docker stop "rover-ros-gateway" 2>/dev/null || true
    docker rm "rover-ros-gateway" 2>/dev/null || true

    echo "Starting rover-ros-gateway container (ROS ${ROS_DISTRO})..."

    docker run --rm -it ${DAEMON_ARG} \
        --name rover-ros-gateway \
        --network host \
        --ipc host \
        --pid host \
        --cap-add=NET_ADMIN \
        rover-ros-gateway:"${ROS_DISTRO}" \
        --interface "${CAN_INTERFACE}" \
        --channel "${CAN_CHANNEL}" \
        --bitrate "${CAN_BITRATE}" \
        --log-level "${ROS_LOG_LEVEL}"

    if [[ -n ${DAEMON_ARG} ]]; then
        echo ""
        echo "Container name: rover-ros-gateway"
        echo "To view logs: docker logs rover-ros-gateway"
        echo "To stop container: docker stop rover-ros-gateway"
        echo "To check status: docker ps --filter name=rover-ros-gateway"
    fi
}

main() {
    parse_arguments "$@"
    setup_can_interface
    run_container
}

main "$@"
