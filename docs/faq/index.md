# Frequently Asked Questions

Here we list common questions that our users have. This list is updated periodically, so if you cannot find what you are looking for, please contact [support@canedudev.com](mailto:support@canedudev.com).

## General Questions

/// details | What is the CanEduDev Rover?
    type: question

The Rover is a sophisticated 1:5 scale model car designed for educational, research, and prototyping applications. It's a distributed embedded system platform featuring multiple STM32F302 microcontrollers communicating via CAN bus, with real-time operation using FreeRTOS and ROS2 integration for high-level control.
///

/// details | What can I do with the Rover?
    type: question

The Rover is designed for:

- Education and development in CAN-based communication systems
- Real-time control systems research
- Robotics and autonomous vehicle prototyping
- Learning about distributed embedded systems
- Integrating with ROS2 for advanced robotics applications
///

/// details | Is the Rover software open source?
    type: question

Yes! The Rover's software is open source and hosted on [GitHub](https://github.com/CanEduDev/rover). You can review, modify, and contribute to the codebase. The project includes firmware, ROS2 gateway, Python tools, and comprehensive documentation. Everything is licensed under the MIT license.
///

/// details | What operating systems are supported for development?
    type: question

The Rover project is primarily developed and tested on **Ubuntu 24.04 LTS**. Windows users can utilize **Ubuntu 24.04 WSL (Windows Subsystem for Linux)**. The bootstrap script handles automatic installation of all required dependencies.
///

## Getting Started

/// details | What battery does the Rover use?
    type: question

The Rover supports **2S-4S LiPo batteries** with an XT60 connector, though **4S is recommended for best performance**. You need to provide your own battery as it's not included with the Rover.
///

/// details | How do I install the battery?
    type: question

1. Remove the top cover of the Rover (mounted with dome nuts)
2. Place your LiPo battery in the battery slot
3. Connect both battery connectors to the power module (main power and cell balance connector)
4. For first-time setup, place the Rover on a box with wheels suspended before turning it on

See the [Getting Started guide](../getting-started/index.md) for detailed instructions with images.
///

/// details | How do I turn the Rover on and off?
    type: question

Press the **main power switch** to turn the system on or off. When powered on, you should see various green LED lights illuminate. Some Rover models have an ESC with a separate power switch that also needs to be pressed after the main power is on.
///

/// details | Why should I disconnect the battery when not in use?
    type: question

If the Rover will be inactive for more than a couple of hours, disconnect the LiPo battery to avoid over-discharging, which can cause irreparable damage. LiPo batteries should never be discharged below 3.0 volts per cell.

///
## Calibration and Maintenance

/// details | The Rover isn't driving straight. What should I do?
    type: question

The Rover comes pre-calibrated from the factory, so in most cases you shouldn't need to adjust it. However, keep in mind:

- The steering mechanism has some natural play in it
- The Rover does not have auto-correction for steering (it won't automatically straighten out)
- Small deviations are normal and expected

If you believe the calibration is significantly off or you've accidentally reset it, see the [Calibration Guide](../getting-started/calibration.md) for detailed instructions on how to recalibrate your Rover.
///

/// details | When should I recalibrate my Rover?
    type: question

You should recalibrate your Rover if:

- You've accidentally reset the calibration by holding SW2 for more than 5 seconds
- The steering or throttle endpoints are not reaching their full range
- You've switched to a different radio transmitter
- The neutral position is significantly off

See the [Calibration Guide](../getting-started/calibration.md) for step-by-step instructions.
///

/// details | How do I calibrate my Rover?
    type: question

Follow the detailed step-by-step instructions in the [Calibration Guide](../getting-started/calibration.md).
///

## Battery and Power

/// details | What is the low-voltage cutoff feature?
    type: question

The Rover's power module includes a low-voltage cutoff feature that's enabled by default. When battery voltage is deemed too low, the power module will turn off most systems to protect your battery from over-discharge damage.
///

/// details | How should I store LiPo batteries?
    type: question

Store LiPo batteries at a safe voltage level, typically around **3.7 to 3.8 volts per cell**. For extended storage, use your charger's storage mode to set the battery to the appropriate voltage. Never store batteries fully charged or fully discharged.
///

/// details | What voltage should I never go below with LiPo batteries?
    type: question

LiPo batteries should **never be discharged below 3.0 volts per cell**. Over-discharging can cause significant capacity loss, performance degradation, and in some cases render the battery completely unusable.
///

## ROS2 Gateway

/// details | What is the ROS2 Gateway?
    type: question

The ROS2 Gateway is a bridge between the Rover's CAN bus and ROS2. It allows you to control the rover using standard ROS2 topics, monitor system status, and integrate with the ROS2 ecosystem for autonomous behaviors. See the [ROS2 documentation](../ros2/index.md) for details.
///

/// details | How do I start the ROS2 Gateway?
    type: question

Use the provided script to run the gateway in a Docker container:

```bash
# Run interactively
./scripts/run-ros-gateway.sh

# Run as background daemon
./scripts/run-ros-gateway.sh --daemon
```

The script automatically configures the CAN interface and starts all ROS2 nodes. See the [ROS2 Gateway documentation](../ros2/index.md) for manual setup instructions.
///

/// details | How do I control the Rover with ROS2?
    type: question

Use the `/rover/cmd_vel` topic with `geometry_msgs/Twist` messages:

```bash
# Move forward at 50% throttle
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn right while moving forward (positive angular.z = right)
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.349}}"

# Turn left while moving forward (negative angular.z = left)
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: -0.349}}"

# Stop
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**Note:** `angular.z` is in radians. Positive values turn right, negative values turn left.
///

/// details | What ROS2 topics are available?
    type: question

**Control topics:**

- `/rover/cmd_vel` - Control commands for throttle and steering

**Monitoring topics** (read-only):

- `/rover/radio/cmd_vel` - Physical radio commands (when radio active)
- `/rover/battery_monitor_*/cell_voltages` - Individual cell voltages
- `/rover/battery_monitor_*/output` - Battery output voltage and current
- `/rover/battery_monitor_*/regulated` - Regulated output voltage and current
- `/rover/wheel_*/wheel_status` - Wheel speeds and RPM
- `/rover/obstacle_detector_*/obstacle_distance` - Distance readings
- `/rover/mayor/can_status` - CAN bus status

**Configuration topics:**

- `/rover/wheel_*/report_frequency_hz` - Configure wheel reporting frequency

See the [Messages and Topics documentation](../ros2/messages.md) for complete details.
///

/// details | What ROS2 nodes run in the gateway?
    type: question

The gateway launches multiple nodes:

- **mayor_node** - CAN Kingdom protocol coordinator
- **controller_node** - Vehicle control with safety override
- **battery_node** (2 instances) - Battery monitoring for control and AD systems
- **wheel_node** (4 instances) - Wheel speed monitoring
- **obstacle_detector_node** (2 instances) - Front and rear distance sensing

See the [Nodes documentation](../ros2/nodes.md) for details on each node.
///

/// details | What ROS2 versions are supported?
    type: question

The gateway supports both **ROS2 Humble** (Ubuntu 22.04) and **ROS2 Jazzy** (Ubuntu 24.04). The Docker images are tagged accordingly (e.g., `ros-gateway-humble` or `ros-gateway-jazzy`).
///

/// details | How does the safety override work?
    type: question

When the physical radio transmitter's override button is active, it **always takes precedence** over ROS2 control commands. The controller node detects radio activity on the CAN bus and stops sending autonomous commands. You can monitor radio commands on the `/rover/radio/cmd_vel` topic. See the [Safety documentation](../ros2/safety.md) for more details.
///

## Hardware and Architecture

/// details | What microcontroller does the Rover use?
    type: question

All Rover modules use the **STM32F302** microcontroller on a CPU board. This provides essential peripherals like UART, CAN, and SWD interfaces, plus SPI flash memory for storing application data.
///

/// details | What is the modular board architecture?
    type: question

The Rover uses a modular design where a **CPU board** (STM32F302) combines with different **carrier boards** to create development boards.
Each development board + application combination creates a module (Power, Servo, Motor, RC, etc.). See the [Rover System documentation](../getting-started/rover-system.md) for architecture details.
///

/// details | What communication protocol does the Rover use?
    type: question

The Rover uses a custom **CAN Kingdom protocol** at **125 kbps** bitrate. The protocol coordinates communication between all components, with concepts like King, Mayor, Postmaster, Cities, and Envelopes organizing the system.
///

/// details | What hardware do I need for development?
    type: question

- Rover board(s) with STM32F302
- ST-Link V2 or V3 programmer/debugger
- USB cable
- Kvaser Leaf or other SocketCAN-compatible interface
- CAN cables and terminators (120Ω resistors)
- 4S LiPo battery with XT60 connector
///

/// details | What modules come with the Rover?
    type: question

**Base modules** (included with all setups):

- Power Module - Power distribution and monitoring
- Servo Module - Steering servo control
- Motor Module - ESC/motor control
- RC Module - Radio control interface

**Optional modules:**

- Wheel Speed Module - Wheel speed data
- FlexiRange - Ultrasonic distance sensing
- Light Array - CAN-controllable LED lights
///

/// details | What CAN messages does the Rover use?
    type: question

The Rover uses a variety of CAN messages for control and monitoring. See the [CAN Messages documentation](../can/index.md) for complete details on all message types.
///

/// details | Where can I find the CAN message definitions?
    type: question

CAN message definitions are documented in the [CAN Messages Index](../can/index.md).

The raw definitions are also available in the `rover.dbc` file in the repository.

///
## Firmware and Updates

/// details | How do I upgrade the Rover's firmware?
    type: question

See the [Firmware Upgrade Guide](../getting-started/upgrading.md) for detailed instructions.
///

/// details | How do I know what firmware version I have?
    type: question

Currently, there is no programmatic way to check the firmware version from the Rover itself. This feature is on our roadmap for future releases.

For now, the best way to track your firmware version is to:

- Keep note of which release you flashed from [Rover Releases](https://github.com/CanEduDev/rover/releases)
- Check the date you last performed an upgrade
- When in doubt, flash the latest version. We always try to maintain backwards compatibility.
///

/// details | What should I do if a firmware upgrade fails?
    type: question

If flashing over CAN fails:

1. Verify CAN interface is properly configured and connected
1. Check that the Rover is powered on and communicating
1. Try power cycling the Rover
1. Use the SWD method as a fallback

If flashing over SWD fails:

1. Verify ST-Link connection and drivers
1. Check that you're using the correct binary file
1. Ensure the start address is set to `0x08000000`
1. Try a different USB port or cable
///

## Troubleshooting

/// details | The Rover doesn't turn on. What should I check?
    type: question

1. Check battery is fully charged and connected to both connectors (main power + cell balance)
2. Ensure main power switch is pressed
3. Check for any loose connections
4. Verify battery voltage is above minimum threshold (3.0V per cell)
5. If using an ESC with separate power switch, ensure it's also pressed
///

/// details | My CAN interface isn't working. How do I fix it?
    type: question

1. Check the interface is properly connected
2. Set up the CAN interface with correct parameters:
   ```bash
   sudo ip link set can0 type can bitrate 125000 restart-ms 100
   sudo ip link set can0 up
   ```
3. Verify with `ip link show can0`
4. Check termination resistors (120Ω at each end of CAN bus)
5. Ensure you have the correct permissions (user in `dialout` and `netdev` groups)
///

/// details | How do I monitor CAN bus traffic?
    type: question

```bash
# Monitor raw CAN messages
candump can0

# Or use the Python tools
python rover_py/candump.py
python rover_py/can-log-decoder.py
```
///

/// details | I can't connect to the ROS2 gateway. What's wrong?
    type: question

Check the following:

1. CAN interface is properly configured and up
2. Gateway version matches your firmware version
3. Rover is powered on and connected
4. Docker container is running: `docker ps --filter name=rover-ros-gateway`
5. Check logs: `docker logs rover-ros-gateway`
6. Verify network settings (host, ipc, pid modes)
///

/// details | The ROS2 gateway nodes aren't publishing data. What's wrong?
    type: question

1. Verify the Rover is powered on and all modules are functioning
2. Check CAN bus activity: `candump can0`
3. Ensure firmware is flashed and configured correctly
4. Check individual node logs in the gateway container
5. Verify CAN bitrate matches (125000 bps)
6. Check that CAN bus has proper termination
///

## Safety

/// details | Is the Rover safe to use?
    type: question

The Rover is not a toy and requires careful handling. Important safety considerations:

- **Battery safety:** LiPo batteries can be dangerous if mishandled
- **Electrical hazards:** Faulty wiring can cause injuries or fire
- **Moving parts:** Keep hands clear of wheels and drivetrain when powered
- **Always test safely:** Use the included carrying case for transport and suspend wheels when first testing

CanEduDev assumes no responsibility for damages due to incorrect handling.
///

/// details | What safety features does the Rover have?
    type: question

- **Low-voltage cutoff:** Protects battery from over-discharge
- **Overcurrent protection:** Customizable limits to prevent damage
- **Reverse polarity protection:** Prevents damage from incorrect battery connection
- **100A fuse:** Hardware protection against excessive current
- **Radio override:** Physical radio can override autonomous control for safety

See the [Safety documentation](../ros2/safety.md) for details on the ROS2 safety architecture.
///

---

## Still Have Questions?

If you cannot find the answer to your question here, please:

- Check the [Getting Started Guide](../getting-started/index.md)
- Browse the [Rover System Documentation](../getting-started/rover-system.md)
- Review the [ROS2 Gateway Documentation](../ros2/index.md)
- Contact support: [support@canedudev.com](mailto:support@canedudev.com)
