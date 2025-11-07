# Rover

This repository contains software and tools for the CanEduDev Rover platform.

The Rover system includes:
- **Firmware**: Embedded C-based firmware for STM32F302 microcontrollers, providing real-time control and CAN communication for all hardware components.
- **ROS2 Gateway**: Python-based bridge between ROS and Rover's CAN system for high-level control.
- **Python Tools**: Development, testing, and configuration utilities.

📚 **[Complete Documentation](https://docs.canedudev.com)** - Full documentation including guides, API references, and tutorials.

## Hardware Components

The Rover has a distributed control system consisting of multiple applications:
- **Battery Monitor**: Monitors battery cell voltages and system power.
- **Light Array**: LED control for lighting.
- **Obstacle Detector**: Distance sensing using ultrasonic sensors.
- **Wheel**: Wheel speed monitoring.
- **Servo**: Steering control.
- **SBUS Receiver**: Adapter for interfacing with hobby Radio systems supporting SBUS.
- **Motor**: Controls hobby ESCs.

## Getting Started

This project is built using [Meson](https://mesonbuild.com/) with the Ninja backend.

**Supported OS**: Ubuntu 24.04. If you're using Windows, you can utilize the Ubuntu 24.04 [WSL](https://learn.microsoft.com/en-us/windows/wsl/) distribution.

### Prerequisites

1. Run `./scripts/bootstrap.sh` to set up the build environment. This step installs all dependencies and prepares the build folder.
2. Enter virtual environment: `source .venv/bin/activate`

It is required to activate the virtual environment to run most tasks in the repo. As such, all the next sections assume the environment has been sourced.
```
source .venv/bin/activate
```

## Building and Running

### Firmware Build
Execute `meson compile -C build` to build all firmware binaries. The build output can be found in the `build` folder.

### Available Build Targets
- **`meson compile -C build`**: Build all firmware binaries
- **`meson compile -C build docs`**: Build code documentation (HTML output in `build/docs/html`)
- **`meson compile -C build release`**: Generate release package with binaries and configuration
- **`meson compile -C build ros-gateway`**: Build ROS2 gateway Docker container
- **`meson test -C build`**: Run all unit tests
- **`meson compile -C build check`**: Run formatters and linters

## ROS2 Gateway

The ROS2 gateway provides a bridge between the Rover's CAN bus and ROS2 topics, enabling high-level control and monitoring. The gateway is deployed as a Docker container.

### Running from GitHub (Recommended)

You can run the gateway directly from GitHub without building locally.

**Prerequisites**
- Docker installed.
- SocketCAN interface set up outside of Docker (e.g., `sudo ip link set can0 up type can bitrate 125000 restart-ms 100`)

**Important**: The gateway version must match your Rover firmware version.

Replace `jazzy` with `humble` in the image name below if you are using ROS2
Humble instead of Jazzy. Similarly, replace `v0.14.0` with your exact firmware
version. See [Github Packages](https://github.com/orgs/CanEduDev/packages?repo_name=rover)
for the different container images.

```bash
# Start the container
docker run --rm -d \
    --name rover-ros-gateway \
    --network host \
    --ipc host \
    --pid host \
    ghcr.io/canedudev/rover/ros-gateway-jazzy:v0.14.0 \
    --interface socketcan \
    --channel can0 \
    --bitrate 125000

# Inspect the container
docker logs rover-ros-gateway

# Stop the container
docker stop rover-ros-gateway
```

### Building and Running Locally

```bash
# Build the gateway
meson compile -C build ros-gateway

# Run the gateway
./scripts/run-ros-gateway.sh
```

The gateway provides control via `/rover/cmd_vel` topic and publishes sensor data from all Rover components. For detailed documentation including topics, safety features, and advanced usage, see [ros2/README.md](ros2/README.md).

## Development Tools

### Python Tools
The project includes comprehensive Python tools for development, testing, and configuration:

- **`rover_py/rover/`**: Python library for Rover interaction
- **`rover_py/flasher/`**: Firmware flashing utilities
- **`rover_py/fw_update.py`**: System-wide firmware update tool
- **`rover_py/can-log-decoder.py`**: CAN bus log analysis
- **`rover_py/calibrate_battery_monitor.py`**: Battery monitor calibration

### Demo Applications
Example applications are available in `rover_py/demo/`:
- **`light-array-demo.py`**: LED array control demonstration
- **`live-signal-plot-demo.py`**: Real-time signal plotting

### Testing
- **Unit Tests**: Run `meson test -C build` for all unit tests
- **Integration Tests**: Located in `rover_py/tests/` and require hardware with CAN interface support (e.g., [Kvaser Leaf Light](https://www.kvaser.com/product/kvaser-leaf-light-hs-v2/))

```bash
# Run integration tests
python rover_py/tests/test_battery_conf.py
python rover_py/tests/test_servo_conf.py
python rover_py/tests/test_wheel_conf.py
```

## Flashing Firmware

### Using OpenOCD (Recommended)

Flash firmware using the build system's OpenOCD targets:

```bash
# Flash individual boards
meson compile -C build swd-flash-battery-monitor
meson compile -C build swd-flash-servo
meson compile -C build swd-flash-motor
meson compile -C build swd-flash-sbus-receiver
```

After flashing firmware, you must also flash the configuration via CAN:

```bash
cd build/release
python fw_update.py -i socketcan -c can0 system
```

For detailed flashing instructions including prerequisites, alternative methods, and troubleshooting, see the [CanEduDev Documentation](https://www.canedudev.com/all-resources?resource_id=1018665).

## Communication Protocol

The Rover uses a custom "CAN Kingdom" protocol where:
- **King**: Coordinates device assignments and system configuration
- **Mayor**: Manages communication between devices
- **Postmaster**: Handles message routing
- **Cities**: Individual hardware components
- **Envelopes**: Message types (steering, throttle, battery, etc.)

CAN bus operates at 125kbps with message types defined in `rover.dbc`.

## Project Structure

- **`apps/`**: Firmware for each hardware component
- **`libs/`**: Shared libraries (CAN Kingdom, STM32 common, etc.)
- **`ros2/`**: ROS2 gateway implementation
- **`rover_py/`**: Python development tools and tests
- **`boards/`**: STM32CubeMX project files
- **`bootloader/`**: Bootloader firmware
- **`docs/`**: Documentation source files
- **`scripts/`**: Build and utility scripts

## Using STM32CubeMX to Generate Code

The hardware initialization code for various boards was initially generated using [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). STM32CubeMX projects are defined in `.ioc` files located in the `boards` directory. These are not actively maintained but can be used as a reference for new applications.

Follow these steps to generate code:
1. Load a project in STM32CubeMX.
2. Click "GENERATE CODE" in the top right corner.
3. If prompted to download firmware, proceed with the download.
