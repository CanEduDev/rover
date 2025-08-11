#!/bin/bash
set -eo pipefail

# Global variables
CAN_INTERFACE="socketcan"
CAN_CHANNEL="can0"
CAN_BITRATE="125000"
ROS_LOG_LEVEL="info"
INTERACTIVE=true
TIMEOUT=30              # 30 seconds default timeout for non-interactive mode
HEALTH_CHECK_INTERVAL=5 # Check container health every 5 seconds

# Function to display help message
show_help() {
    echo "Usage: $0 [--interface <can_interface>] [--channel <can_channel>] [--bitrate <can_bitrate>] [--log-level <log_level>] [--non-interactive] [--timeout <seconds>]"
    echo
    echo "Options:"
    echo "  -i, --interface <can_interface>   CAN interface to use (default: socketcan)"
    echo "  -c, --channel <can_channel>       CAN channel to use (default: can0)"
    echo "  -b, --bitrate <can_bitrate>       CAN bitrate to use (default: 125000)"
    echo "  -l, --log-level <log_level>       ROS2 log level (default: info)"
    echo "  --non-interactive                 Run in non-interactive mode (for automated execution)"
    echo "  --timeout <seconds>               Timeout in seconds for non-interactive mode (default: 30)"
    echo "  -h, --help                    Show this help message and exit"
    echo
    echo "Supported ROS2 log levels:"
    echo "  UNSET, DEBUG, INFO, WARN, ERROR, FATAL"
    echo
    echo "Examples:"
    echo "  $0 --interface socketcan --channel can0 --bitrate 125000 --log-level info"
    echo "  $0 --log-level warn"
    echo "  $0 --non-interactive --log-level debug --timeout 60"
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
        --non-interactive)
            INTERACTIVE=false
            shift
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        -h | --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--interface <can_interface>] [--channel <can_channel>] [--bitrate <can_bitrate>] [--log-level <log_level>] [--non-interactive] [--timeout <seconds>]"
            echo "Try '$0 --help' for more information."
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

# Function to check if container is running and healthy
check_container_health() {
    local container_name="$1"
    local logs

    # Check if container exists and is running
    if ! docker ps --format "table {{.Names}}" | grep -q "^${container_name}$"; then
        echo "Container ${container_name} is not running"
        return 1
    fi

    # Check container logs for error patterns
    local error_patterns=("ERROR" "FATAL" "Exception" "Traceback" "failed to start" "connection refused")
    logs=$(docker logs --tail 50 "${container_name}" 2>&1 || true)

    for pattern in "${error_patterns[@]}"; do
        if echo "${logs}" | grep -qi "${pattern}"; then
            echo "Container ${container_name} shows error pattern: ${pattern}"
            return 1
        fi
    done

    return 0
}

# Function to gracefully stop container
stop_container() {
    local container_name="$1"
    echo "Stopping container ${container_name}..."
    docker stop "${container_name}" 2>/dev/null || true
    docker rm "${container_name}" 2>/dev/null || true
}

# Function to build docker run command
build_docker_command() {
    DOCKER_CMD="docker run --rm"
    if [[ ${INTERACTIVE} == "true" ]]; then
        DOCKER_CMD="${DOCKER_CMD} -it"
    fi
}

# Function to run container in interactive mode
run_interactive_container() {
    echo "Starting rover-ros-gateway container in interactive mode..."
    ${DOCKER_CMD} \
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
}

# Function to run container in non-interactive mode with monitoring
run_noninteractive_container() {
    echo "Starting rover-ros-gateway container in non-interactive mode (timeout: ${TIMEOUT}s)..."

    # Start container in background
    ${DOCKER_CMD} \
        --name rover-ros-gateway \
        --network host \
        --ipc host \
        --pid host \
        --cap-add=NET_ADMIN \
        rover-ros-gateway \
        --interface "${CAN_INTERFACE}" \
        --channel "${CAN_CHANNEL}" \
        --bitrate "${CAN_BITRATE}" \
        --log-level "${ROS_LOG_LEVEL}" &

    CONTAINER_PID=$!

    # Wait for container to start
    sleep 5

    # Monitor container health with timeout
    start_time=$(date +%s)
    while true; do
        current_time=$(date +%s)
        elapsed=$((current_time - start_time))

        # Check timeout
        if [[ ${elapsed} -ge ${TIMEOUT} ]]; then
            echo "Timeout reached (${TIMEOUT}s). Stopping container..."
            stop_container "rover-ros-gateway"
            kill "${CONTAINER_PID}" 2>/dev/null || true
            exit 124 # Timeout exit code
        fi

        # Check container health
        # shellcheck disable=SC2310
        if ! check_container_health "rover-ros-gateway"; then
            echo "Container health check failed. Stopping container..."
            stop_container "rover-ros-gateway"
            kill "${CONTAINER_PID}" 2>/dev/null || true
            exit 1
        fi

        # Check if container process has exited
        if ! kill -0 "${CONTAINER_PID}" 2>/dev/null; then
            echo "Container process has exited"
            break
        fi

        sleep "${HEALTH_CHECK_INTERVAL}"
    done

    # Wait for background process and get exit code
    wait "${CONTAINER_PID}"
    return $?
}

# Main function
main() {
    # Parse command line arguments
    parse_arguments "$@"

    # Set up CAN interface
    setup_can_interface

    # Build docker command
    build_docker_command

    # Run container based on interactive mode
    if [[ ${INTERACTIVE} == "false" ]]; then
        run_noninteractive_container
        EXIT_CODE=$?
    else
        run_interactive_container
        EXIT_CODE=$?
    fi

    echo "Container exited with code: ${EXIT_CODE}"
    exit "${EXIT_CODE}"
}

# Call main function with all arguments
main "$@"
