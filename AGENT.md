# Rover Project - AI Assistant Guide

## Project Overview

The Rover project is a comprehensive embedded systems codebase for the CanEduDev Rover, featuring multiple STM32F302-based boards communicating over CAN bus. The system includes firmware for various hardware components, a ROS2 gateway for high-level control, and Python tools for development and testing.

## Architecture

### Hardware Components
- **Battery Monitor**: Monitors battery cell voltages and system power
- **Light Arrays** (Front/Rear): LED control for lighting
- **Obstacle Detectors** (Front/Rear): Distance sensing
- **Wheels** (4x): Speed monitoring and control
- **Servo**: Steering control
- **SBUS Receiver**: Radio control input
- **Motor**: Throttle control

### Software Architecture
- **Firmware**: C-based embedded code using FreeRTOS
- **CAN Kingdom Protocol**: Custom communication protocol for device coordination
- **ROS2 Gateway**: Python-based bridge between ROS and CAN bus
- **Python Tools**: Development, testing, and configuration utilities

## Key Technologies

### Build System
- **Meson**: Primary build system with Ninja backend
- **STM32CubeMX**: Hardware initialization code generation
- **FreeRTOS**: Real-time operating system
- **CAN Bus**: Communication protocol (125kbps)

### Development Tools
- **Python 3.12+**: Scripting and tools
- **OpenOCD**: Debugging and flashing
- **Kvaser or SocketCAN CAN interfaces**: Hardware communication
- **STM32CubeProgrammer**: Firmware flashing

## Project Structure

### Core Directories

#### `/apps/` - Firmware Applications
Each subdirectory contains firmware for a specific hardware component:
- `battery-monitor/`: Battery monitoring and power management
- `light-array/`: LED array control
- `obstacle-detector/`: Distance sensor processing
- `servo/`: Steering servo control
- `sbus-receiver/`: Radio control receiver
- `wheel/`: Wheel speed monitoring

#### `/libs/` - Shared Libraries
- `can-kingdom/`: CAN communication protocol implementation
- `rover/`: Rover-specific assignments and coordination
- `stm32-common/`: STM32F302 HAL and common utilities
- `circuits/`: Hardware circuit abstractions
- `float/`: Floating-point utilities
- `json/`: JSON parsing for configuration
- `arena/`: Memory allocation utilities

#### `/ros2/` - ROS2 Gateway
- `src/gateway/`: Python ROS2 nodes for CAN communication
- `src/gateway_msgs/`: Custom ROS2 message definitions
- Gateway nodes: controller, battery, wheel, obstacle detector, mayor, radio

#### `/rover_py/` - Python Development Tools
- `rover/`: Python library for Rover interaction
- `flasher/`: Firmware flashing utilities
- `tests/`: Integration tests
- `demo/`: Example applications

#### `/boards/` - Hardware Configuration
- `.ioc` files for STM32CubeMX project configuration

## Communication Protocol

### CAN Kingdom Protocol
The system uses a custom "CAN Kingdom" protocol where:
- **King**: Coordinates device assignments and system configuration
- **Mayor**: Manages communication between devices
- **Postmaster**: Handles message routing
- **Cities**: Individual hardware components
- **Envelopes**: Message types (steering, throttle, battery, etc.)

### CAN Message Types (from rover.dbc)
- **256**: STEERING - Steering angle and mode
- **257**: THROTTLE - Throttle control
- **288-289**: LIGHT_ARRAY_STATE - LED status
- **512**: BATTERY_CELL_VOLTAGES - Individual cell voltages
- **513-517**: Power monitoring messages
- **518**: SERVO_POSITION - Servo feedback
- **528-531**: WHEEL_SPEED - Wheel speed data

## Development Workflow

Assume all commands require the Python venv to be activated.

```
source .venv/bin/activate
```

### Building Firmware
```bash
# Setup environment
./scripts/bootstrap.sh
source .venv/bin/activate

# Build all firmware
meson compile -C build

# Build specific component
meson compile -C build battery-monitor
```

### Building ROS2 Gateway
```bash
# Build ROS2 workspace
meson compile -C build

# Or build specific ROS2 components
meson compile -C build ros-gateway
```

### Running Tests
```bash
# Unit tests
meson test -C build

# Integration tests (requires Kvaser hardware)
python rover_py/tests/test_battery_conf.py
```

### Running ROS2 Gateway
```bash
# Non-interactive mode with timeout (for reading logs):
./scripts/run-ros-gateway.sh --non-interactive

# Run as daemon in background:
./scripts/run-ros-gateway.sh --daemon

# Check if daemon is running
docker ps --filter name=rover-ros-gateway

# View daemon logs
docker logs rover-ros-gateway

# Stop daemon
docker stop rover-ros-gateway

# Running generic ros2 commands in the container:
# After running as daemon, use:
docker exec rover-ros-gateway bash -c "source install/setup.bash && ros2 [CMD]"

```

### Flashing Firmware
```bash
# Via STM32CubeProgrammer (SWD)
# Load binary to board via ST-Link

# Via CAN (requires working system)
python fw_update.py system --config config/system.json
```

## Key Development Patterns

### Firmware Structure
Each app follows a consistent pattern:
- `include/`: Header files
- `src/`: Source files with main.c entry point
- `tests/`: Unit tests (when applicable)
- `meson.build`: Build configuration

### Common Libraries Usage
- **STM32 Common**: HAL, FreeRTOS integration, error handling
- **CAN Kingdom**: Communication protocol
- **Rover Helpers**: Rover-specific utilities

### ROS2 Gateway Patterns
- **Node-per-Component**: Each hardware component has a corresponding ROS2 node
- **CAN Message Translation**: Bidirectional conversion between CAN and ROS2 messages
- **Threading**: Separate threads for CAN reading and ROS2 publishing

## Configuration and Calibration

### System Configuration
- Generated via `scripts/gen-system-conf.py`
- Stored in JSON format
- Includes device assignments and calibration data

### Battery Calibration
- `rover_py/calibrate_battery_monitor.py`
- Calibrates voltage dividers and cell monitoring

### Servo Calibration
- `rover_py/tests/test_servo_conf.py`
- Calibrates steering servo endpoints

## Debugging and Monitoring

### CAN Bus Monitoring
```bash
# Monitor CAN traffic
python rover_py/candump.py

# Decode CAN messages
python rover_py/can-log-decoder.py
```

### ROS2 Topics
- `/cmd_vel`: Velocity commands (geometry_msgs/Twist)
- `/battery_status`: Battery information
- `/wheel_status`: Wheel speed data
- `/obstacle_distance`: Distance sensor readings

### Firmware Debugging
- OpenOCD configuration in `openocd.cfg`
- STM32F302 SVD file for register definitions
- FreeRTOS task monitoring

## Common Development Tasks

### Adding New Hardware Component
1. Create new app directory in `/apps/`
2. Implement CAN Kingdom city protocol
3. Add to rover assignments in `libs/rover/`
4. Update `rover.dbc` with new message definitions
5. Create corresponding ROS2 node if needed

### Modifying CAN Protocol
1. Update `rover.dbc` file
2. Regenerate Python bindings
3. Update firmware message handling
4. Update ROS2 gateway nodes

### Adding New ROS2 Functionality
1. Define messages in `ros2/src/gateway_msgs/`
2. Implement node in `ros2/src/gateway/gateway/`
3. Update launch files and dependencies

## Testing Strategy

### Unit Tests
- C-based tests using FFF (Fake Function Framework)
- Meson test framework
- Hardware abstraction for testing

### Integration Tests
- Python-based tests requiring actual hardware
- CAN bus communication testing
- End-to-end system validation

### Hardware-in-the-Loop
- Kvaser CAN interfaces for testing
- Real hardware validation
- Performance and reliability testing

## Performance Considerations

### Real-time Requirements
- FreeRTOS task priorities
- CAN message timing
- Interrupt handling

### Memory Management
- Static allocation preferred
- Arena allocator for dynamic memory
- LittleFS for persistent storage

### Power Management
- Battery monitoring and protection
- Voltage regulation monitoring

## Troubleshooting

### Common Issues
1. **CAN Communication**: Check bitrate, termination, and device IDs
2. **Firmware Flashing**: Verify ST-Link connection and binary compatibility
3. **ROS2 Gateway**: Check CAN interface configuration and permissions
4. **Build Issues**: Ensure virtual environment is activated and dependencies installed

### Debug Tools
- `rover_py/candump.py`: CAN bus monitoring
- `rover_py/can-log-decoder.py`: Message decoding
- OpenOCD: Firmware debugging
- ROS2 tools: Topic monitoring and debugging

## Best Practices

### Code Style
- Follow existing C coding standards
- Use consistent naming conventions
- Include comprehensive error handling
- Add unit tests for new functionality

### Documentation
- Update header file documentation
- Maintain README files
- Document CAN message formats
- Keep build instructions current

### Version Control
- Use semantic versioning
- Tag releases appropriately
- Maintain changelog
- Test before releasing

This guide should help AI coding tools understand the Rover project structure and assist with development tasks effectively.
