# ROS2 Gateway Workspace

This ROS2 workspace provides a gateway between the CAN bus and ROS2 topics, converting CAN messages to and from ROS2 messages for the CanEduDev Rover.

## Workspace Structure

The workspace contains two main packages:

### `gateway_msgs/` - Message Definitions
Custom ROS2 message types for the rover's various subsystems:
- `BatteryStatus.msg` - Battery status with cell voltages, output and regulated voltage/current
- `WheelStatus.msg` - Wheel speed and RPM information
- `ObstacleDistance.msg` - Distance readings from obstacle detectors
- `ReportFrequency.msg` - Report frequency configuration
- `CANStatus.msg` - CAN bus status information

### `gateway/` - Main Gateway Package
ROS2 nodes that act as the gateway between CAN bus and ROS2 topics:
- **battery_node**: Publishes battery status using `gateway_msgs/BatteryStatus`
- **wheel_node**: Publishes wheel status information
- **obstacle_detector_node**: Publishes obstacle distance readings
- **controller_node**: Subscribes to `/cmd_vel` for control commands
- **radio_node**: Monitors physical radio state for safety override
- **mayor_node**: Publishes CAN bus status

## Safety Architecture

The system implements a sophisticated safety mechanism with human override capability:

### Safety Override System
- **Autonomous/Manual Control Priority**: Normal operation via `/cmd_vel` commands
- **Physical Radio Safety Override**: When the physical radio is active on the CAN bus, it overrides all other control
- **Safety Monitoring**: The controller monitors CAN bus activity to detect radio presence
- **Override Prevention**: Autonomous/manual commands are ignored when physical radio is active

### Control Flow
```
Autonomous/Manual Control → /cmd_vel → Controller Node → CAN Bus (normal operation)
                                                      ↓
Physical Radio → CAN Bus → Radio Node → /cmd_vel (monitoring radio override state)
```

## Containerization

The workspace is containerized using Docker for easy deployment and consistency:

### Docker Structure
- **Dockerfile**: Located at `ros2/Dockerfile` - builds the entire ROS2 workspace
- **Entrypoint**: Located at `ros2/entrypoint.sh` - launches the gateway system
- **Build Context**: Uses the repository root to access both `ros2/` and `rover_py/`

### Building the Container
```bash
# Using the project's build system
meson compile -C build ros-gateway

# Or directly with Docker
docker build -f ros2/Dockerfile -t rover-ros-gateway .
```

### Running the Container
```bash
# Basic usage
docker run --network host rover-ros-gateway

# With custom CAN interface settings
docker run --network host rover-ros-gateway \
  --interface socketcan \
  --channel can0 \
  --bitrate 125000 \
  --log-level info
```

## Message Types

### Standard ROS2 Messages Used
- **`geometry_msgs/Twist`**: Control commands (replaces custom ControlCommand.msg and RadioControl.msg)

### Custom Messages
- **`BatteryStatus.msg`**: Contains battery status with cell voltages, output and regulated voltage/current
- **`WheelStatus.msg`**: Contains wheel speed and RPM information
- **`ObstacleDistance.msg`**: Contains distance readings from obstacle detectors
- **`ReportFrequency.msg`**: Contains report frequency configuration
- **`CANStatus.msg`**: Contains CAN bus status information

## Building and Testing

### Building the Workspace
```bash
# Build entire workspace
cd ros2
colcon build

# Build specific packages
colcon build --packages-select gateway_msgs
colcon build --packages-select gateway
```

### Testing
```bash
# Run tests for the gateway package
colcon test --packages-select gateway

# Run all tests
colcon test
```

## Topic Structure

### Control Topics
- **`/cmd_vel`** (`geometry_msgs/Twist`): Control commands for throttle and steering
  - `linear.x`: Forward/backward velocity (-1 to 1, converted to throttle -100 to 100)
  - `angular.z`: Angular velocity in radians (converted to steering angle in degrees)

### Sensor Topics
- **`/rover/battery_monitor_*/output`** (`gateway_msgs/BatteryStatus`): Battery output voltage and current
- **`/rover/battery_monitor_*/regulated`** (`gateway_msgs/BatteryStatus`): Regulated output voltage and current
- **`/rover/battery_monitor_*/cell_voltages`** (`gateway_msgs/BatteryStatus`): Individual cell voltages
- **`/rover/wheel_*/wheel_status`** (`gateway_msgs/WheelStatus`): Wheel speed and RPM
- **`/rover/obstacle_detector_*/obstacle_distance`** (`gateway_msgs/ObstacleDistance`): Distance readings

### Status Topics
- **`/rover/mayor/can_status`** (`gateway_msgs/CANStatus`): CAN bus status information

## Dependencies

### External Dependencies
- **`geometry_msgs`**: Standard ROS2 geometry messages
- **`launch`**: ROS2 launch system
- **`launch_ros`**: ROS2 launch system extensions
- **`rclpy`**: Python ROS2 client library

### Internal Dependencies
- **`gateway_msgs`**: Custom message definitions (depended on by `gateway`)

### Python Dependencies
- **`python3-can`**: CAN bus communication library
- **`rover_py`**: Rover Python utilities (from the main repository)
