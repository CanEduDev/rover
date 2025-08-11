# Rover

This repository contains the default code for all boards in the CanEduDev Rover - a comprehensive embedded systems platform featuring multiple STM32F302-based boards communicating over CAN bus.

The Rover system includes:
- **Firmware**: C-based embedded code using FreeRTOS for various hardware components
- **ROS2 Gateway**: Python-based bridge between ROS and CAN bus for high-level control
- **Python Tools**: Development, testing, and configuration utilities
- **CAN Kingdom Protocol**: Custom communication protocol for device coordination

For more documentation, visit [CanEduDev's Documentation website](https://www.canedudev.com/getting-started-with-the-rover/).

## Hardware Components

The Rover consists of multiple specialized boards:
- **Battery Monitor**: Monitors battery cell voltages and system power
- **Light Arrays** (Front/Rear): LED control for lighting
- **Obstacle Detectors** (Front/Rear): Distance sensing using ultrasonic sensors
- **Wheels** (4x): Speed monitoring and control
- **Servo**: Steering control with position feedback
- **SBUS Receiver**: Radio control input for manual operation
- **Motor**: Controls hobby ESCs (built from servo firmware with -DMOTOR flag)

## Working with the repository

This project is built using [Meson](https://mesonbuild.com/) with the Ninja backend.

**Supported OS**: Ubuntu 24.04. If you're using Windows, you can utilize the Ubuntu 24.04 [WSL](https://learn.microsoft.com/en-us/windows/wsl/) distribution.

### Prerequisites

1. Run `./scripts/bootstrap.sh` to set up the build environment. This step installs all dependencies and prepares the build folder.
2. Enter virtual environment: `source .venv/bin/activate`

It is required to activate the virtual environment to run most tasks in the repo. As such, all the next sections assume the environment has been sourced.

## Building the Firmware

Execute `meson compile -C build` to initiate the build process. The build output can be found in the `build` folder.

### Available Build Targets

- **`meson compile -C build`**: Build all firmware binaries
- **`meson compile -C build docs`**: Build code documentation (HTML output in `build/docs/html`)
- **`meson compile -C build release`**: Generate release package with binaries and configuration
- **`meson compile -C build ros-gateway`**: Build ROS2 gateway Docker container
- **`meson test -C build`**: Run all unit tests
- **`meson compile -C build check`**: Run formatters and linters

## Building the ROS2 Gateway

The ROS2 gateway provides a bridge between the Rover's CAN bus and ROS2 topics, enabling high-level control and monitoring. The gateway is deployed as a Docker container.

### Quick Start
```bash
# Build the gateway
meson compile -C build ros-gateway

# Run the gateway
./scripts/run-ros-gateway.sh
```

The gateway provides control via `/rover/cmd_vel` topic and publishes sensor data from all Rover components. For detailed documentation including topics, safety features, and advanced usage, see [ros2/README.md](ros2/README.md).

## Python Development Tools

The project includes comprehensive Python tools for development, testing, and configuration:

### Available Tools
- **`rover_py/rover/`**: Python library for Rover interaction
- **`rover_py/flasher/`**: Firmware flashing utilities
- **`rover_py/fw_update.py`**: System-wide firmware update tool
- **`rover_py/can-log-decoder.py`**: CAN bus log analysis
- **`rover_py/calibrate_battery_monitor.py`**: Battery monitor calibration

### Integration Tests
Integration tests are located in `rover_py/tests/` and require hardware with CAN interface support (e.g., Kvaser Leaf Light):

```bash
# Run integration tests
python rover_py/tests/test_battery_conf.py
python rover_py/tests/test_servo_conf.py
python rover_py/tests/test_wheel_conf.py
```

### Demo Applications
Example applications are available in `rover_py/demo/`:
- **`light-array-demo.py`**: LED array control demonstration
- **`live-signal-plot-demo.py`**: Real-time signal plotting

## Building the Code Documentation

To build the documentation, run `meson compile -C build docs`. The HTML output is located in the `build/docs/html` folder.

## Building a Release

Generate a zip file containing the board binaries by running `meson compile -C build release`. The output is stored in the `build` folder.

## Running Tests

To run all unit tests, execute `meson test -C build`.

Additionally, there are integration tests that run against the boards. Note that these tests require hardware that supports canlib, such as the [Kvaser Leaf Light](https://www.kvaser.com/product/kvaser-leaf-light-hs-v2/).

The integration tests are found in `rover_py/tests`. They can be run using `python <path-to-test>.py`.

## Using STM32CubeMX to Generate Code

The hardware initialization code for various boards was initially generated using [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). STM32CubeMX projects are defined in `.ioc` files located in the `boards` directory.

Follow these steps to generate code:
1. Load a project in STM32CubeMX.
2. Click "GENERATE CODE" in the top right corner.
3. If prompted to download firmware, proceed with the download.

## Flashing Binaries onto the Board

There are two steps to flash the binaries:

### Step 1: Initial Flash via SWD (if needed)
Flash the board via SWD using an STM32 programmer such as the [STLINK-V3SET programmer](https://www.st.com/en/development-tools/stlink-v3set.html) along with the [STM32CubeProgrammer software](https://www.st.com/en/development-tools/stm32cubeprog.html). For rovers delivered with working software, this step can be skipped.

### Step 2: System-wide Flash via CAN
Flash the binary and configuration via CAN using a Kvaser or SocketCAN interface:

```bash
# Using the build system target (Kvaser only)
meson run -C build can-flash-all

# Or manually from the release folder
cd build/release
python fw_update.py -i socketcan system --config config/system.json --binary-dir binaries
```

**⚠️ Important**: Do not interrupt the flashing process as this will brick your devices. Bricked devices can be restored by following Step 1.

### Upgrading a working system's firmware
To update a working system, download the latest release and follow Step 2 above.

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

For detailed project architecture and development guidelines, see [AGENT.md](AGENT.md).
