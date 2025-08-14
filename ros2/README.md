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
- **controller_node**: Handles both manual control and radio override functionality
- **mayor_node**: Publishes CAN bus status

## Safety Architecture

The system implements a sophisticated safety mechanism with human override capability:

### Safety Override System
- **Autonomous/Manual Control Priority**: Normal operation via `/rover/cmd_vel` commands
- **Physical Radio Safety Override**: When the physical radio is active on the CAN bus, it overrides all other control
- **Safety Monitoring**: The controller node monitors CAN bus activity to detect radio presence
- **Override Prevention**: Autonomous/manual commands are ignored when physical radio is active
- **Radio Command Publishing**: Radio commands are published to `/radio/cmd_vel` for monitoring

### Control Flow
```
Autonomous/Manual Control → /rover/cmd_vel → Controller Node → CAN Bus (normal operation)
                                                      ↓
Physical Radio → CAN Bus → Controller Node → /radio/cmd_vel (radio override monitoring)
```

## Containerization

The workspace is containerized using Docker for easy deployment and consistency:

### Docker Structure
- **Dockerfile**: Located at `ros2/Dockerfile` - builds the entire ROS2 workspace
- **Entrypoint**: Located at `ros2/entrypoint.sh` - launches the gateway system
- **Build Context**: Uses the repository root to access both `ros2/` and `rover_py/`

### Building and running
From the project's root, run:

```bash
meson compile -C build ros-gateway # build
./scripts/run-ros-gateway.sh # run
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

## Topic Structure

### Control Topics
- **`/rover/cmd_vel`** (`geometry_msgs/Twist`): Control commands for throttle and steering
  - `linear.x`: Forward/backward velocity (-1 to 1), where 1 is full throttle and -1 is full reverse.
  - `angular.z`: Angular velocity in radians
- **`/radio/cmd_vel`** (`geometry_msgs/Twist`): Radio control commands (published by controller node when radio is active)
  - `linear.x`: Radio throttle input (-1 to 1)
  - `angular.z`: Radio steering input in radians

### Sensor Topics
- **`/rover/battery_monitor_*/output`** (`gateway_msgs/BatteryStatus`): Battery output voltage and current
- **`/rover/battery_monitor_*/regulated`** (`gateway_msgs/BatteryStatus`): Regulated output voltage and current
- **`/rover/battery_monitor_*/cell_voltages`** (`gateway_msgs/BatteryStatus`): Individual cell voltages
- **`/rover/wheel_*/wheel_status`** (`gateway_msgs/WheelStatus`): Wheel speed and RPM
- **`/rover/obstacle_detector_*/obstacle_distance`** (`gateway_msgs/ObstacleDistance`): Distance readings

### Status Topics
- **`/rover/can_status`** (`gateway_msgs/CANStatus`): CAN bus status information

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
