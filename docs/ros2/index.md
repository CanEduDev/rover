# ROS2 Gateway

The Rover platform provides a ROS2 gateway that allows you to control and monitor the Rover using standard ROS2 tools and topics.

## What is ROS2?

[ROS2 (Robot Operating System 2)](https://www.ros.org/) is a flexible framework for writing robot software. The Rover ROS2 Gateway acts as a bridge between the rover's CAN bus and ROS2, allowing you to:

- **Control the Rover** using ROS2 messages
- **Monitor the system** check battery state and get wheel speeds
- **Integrate with ROS2 tools** like RViz, Gazebo and custom nodes
- **Build autonomous behaviors** using the ROS2 ecosystem

## Prerequisites

Before running the ROS2 gateway, ensure you have:

1. **Linux computer** with arm64 (e.g. Raspberry Pi, Nvidia Jetson) or x86 (laptop) architecture
1. **Docker** installed on your system
1. **CAN interface** with SocketCAN support
1. **Rover powered on** and connected to your computer via CAN

## Running the Gateway

### Step 1: Configure CAN Interface

Set up your SocketCAN interface:

```bash
sudo ip link set can0 type can bitrate 125000 restart-ms 100
sudo ip link set can0 up
```

/// note | CAN Interface ID
If needed, replace `can0` with your correct CAN interface ID. For example, Nvidia Jetson Nano has `can0` already configured, so if you're using a USB-to-CAN interface such as the Kvaser Leaf, it will register as `can1`.
///

### Step 2: Start the Gateway

Run the pre-built gateway container from GitHub:

```bash
docker run --rm -d \
    --name rover-ros-gateway \
    --network host \
    --ipc host \
    --pid host \
    ghcr.io/canedudev/rover/ros-gateway-jazzy:v0.14.0 \
    --interface socketcan \
    --channel can0 \
    --bitrate 125000
```

/// note | Version Matching
The gateway version must match your Rover firmware version. Replace:

- `jazzy` with `humble` if using ROS2 Humble
- `v0.14.0` with your firmware version
///

/// note | Docker Options
The Docker options `--network host --ipc host --pid host` enable the user to interact with the ROS2 gateway from their host operating system.
///

### Step 3: Verify the Gateway is Running

Check the container logs:

```bash
docker logs rover-ros-gateway
```

You should see messages indicating the gateway has connected to the CAN bus and is publishing topics.

/// tip | Need Help?
If you encounter issues:

- Check that your CAN interface is configured with the proper bitrate.
- Check that your CAN bus is terminated. Measuring with a multimeter between CAN high and CAN low should yield 60 Ohm.
- Verify the gateway version matches your firmware version.
- Ensure the Rover is powered on and connected to the computer running the gateway.
///

## Basic Control

Once the gateway is running, you can control the rover using the `/rover/cmd_vel` topic:

```bash
# Enter the gateway container:
docker exec -it rover-ros-gateway bash

# Source the ROS2 environment:
source install/setup.bash

# Control the rover - move forward at 50% throttle
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left while moving forward
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.5}}"

# Stop the rover
ros2 topic pub /rover/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Monitoring Sensors

View available topics:

```bash
# List all rover topics
ros2 topic list | grep /rover

# Monitor battery status
ros2 topic echo /rover/battery_monitor_front/cell_voltages

# Monitor wheel speeds
ros2 topic echo /rover/wheel_front_left/wheel_status

# Monitor obstacle detectors
ros2 topic echo /rover/obstacle_detector_front/obstacle_distance
```

## Stopping the Gateway

To stop the gateway:

```bash
docker stop rover-ros-gateway
```

## Next Steps

Now that you have the gateway running, you can read deep dive into its features through the following links:

<div class="grid cards" markdown>
-   :material-access-point:{ .lg .middle } **ROS2 Nodes**

    ---

    Complete reference for all gateway nodes

    [:octicons-arrow-right-24: Explore nodes](nodes.md)

-   :material-access-point:{ .lg .middle } **ROS2 Topics**

    ---

    Complete reference for all available topics

    [:octicons-arrow-right-24: Explore topics](messages.md)

-   :material-shield-check:{ .lg .middle } **Safety Features**

    ---

    Understand the radio override system for safe operation

    [:octicons-arrow-right-24: Learn about safety](safety.md)

</div>
