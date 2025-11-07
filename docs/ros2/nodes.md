# Nodes

The Rover gateway consists of multiple ROS2 nodes that bridge CAN bus communication with ROS2 topics. Each node runs in the `/rover` namespace and handles specific hardware components or system functions.

**Architecture:**

- **Single CAN Bus**: All nodes share the same CAN interface
- **Threaded Design**: Each node runs a dedicated CAN reader thread
- **Message Filtering**: Nodes use CAN filters to receive only relevant messages
- **Namespace**: All nodes operate under `/rover/` namespace

## Node Types

There are various node types in the system and there can be more than one instance of a node, for example two wheel nodes when you have two wheel speed modules. This section documents the various types and their instances.

### Mayor Node

**Executable:** `mayor_node`

**Node Name:** `mayor`

**Purpose:** CAN Kingdom protocol coordinator and system status manager

**Responsibilities:**

- Monitors CAN Kingdom "King's Page" commands (CAN ID 0)
- Manages system-wide CAN communication state
- Publishes CAN bus status to other nodes
- Enforces FREEZE/RUN action modes
- Handles LISTEN_ONLY/SILENT/COMMUNICATE communication modes

**Topics Published:**

- `/rover/mayor/can_status` ([gateway_msgs/CANStatus](messages.md#gateway_msgscanstatus)) - CAN bus status

**CAN Messages:**

- Listens to CAN ID 0 (King's Page commands)
- Monitors action mode and communication mode

**Key Features:**

- Disables CAN when in FREEZE action mode
- Disables CAN when in LISTEN_ONLY or SILENT comm modes
- Enables CAN when in RUN action mode with COMMUNICATE comm mode
- Publishes status changes to coordinate other nodes

### Controller Node

**Executable:** `controller_node`

**Node Name:** `controller`

**Purpose:** Vehicle throttle and steering control with safety override

**Responsibilities:**

- Subscribes to ROS2 control commands
- Translates commands to CAN throttle and steering messages
- Monitors physical radio for safety override
- Publishes radio commands when physical radio active
- Enforces safe direction changes (stop before reversing)

**Topics Subscribed:**

- `/rover/cmd_vel` ([geometry_msgs/Twist](messages.md#geometry_msgstwist)) - Control commands
- `/rover/mayor/can_status` ([gateway_msgs/CANStatus](messages.md#gateway_msgscanstatus)) - CAN status

**Topics Published:**

- `/rover/radio/cmd_vel` ([geometry_msgs/Twist](messages.md#geometry_msgstwist)) - Radio override commands

**CAN Messages:**

- Sends: STEERING (256), THROTTLE (257)
- Receives: STEERING (256), THROTTLE (257) for radio monitoring

**Key Features:**

- **Safety Override**: Physical radio always takes precedence
- **Direction Change Protection**: Requires 2-second neutral pause when reversing
- **Input Validation**: Clamps throttle (-1 to 1) and steering (±45°)
- **Control Frequency**: Sends commands at 100 Hz
- **PWM Conversion**: Converts throttle to 1000-2000µs pulse width

**Parameters:**

- None (uses launch arguments for CAN configuration)

### Battery Node

**Executable:** `battery_node`

**Purpose:** Battery monitoring and power management

**Responsibilities:**

- Monitors battery cell voltages (6 cells)
- Tracks battery output voltage and current
- Monitors regulated output voltage and current
- Publishes separate topics for each measurement type

**Topics Published:**

- `/rover/battery_monitor_<position>/cell_voltages` ([gateway_msgs/BatteryStatus](messages.md#gateway_msgsbatterystatus))
- `/rover/battery_monitor_<position>/output` ([gateway_msgs/BatteryStatus](messages.md#gateway_msgsbatterystatus))
- `/rover/battery_monitor_<position>/regulated` ([gateway_msgs/BatteryStatus](messages.md#gateway_msgsbatterystatus))

**CAN Messages:**

- Receives cell voltages (split across two messages)
- Receives output voltage/current
- Receives regulated output voltage/current

**Key Features:**

- **Multi-Message Assembly**: Combines two CAN messages for 6 cell voltages
- **Message Validation**: Ensures correct message reception order
- **Dual Battery Support**: Separate instances for control and AD systems
- **Position-Specific CAN IDs**: Different CAN IDs for each battery position

**Parameters:**

- `position` (string): Battery position
  - `"control_system"` - Control system battery (default)
  - `"ad_system"` - Autonomous driving system battery

**Instances:**

| Node Name | Position | Purpose |
|-----------|----------|---------|
| `battery_monitor_control_system` | `control_system` | Main rover control system power |
| `battery_monitor_ad_system` | `ad_system` | Autonomous driving system power |

### Wheel Node

**Executable:** `wheel_node`

**Purpose:** Wheel speed monitoring and configuration

**Responsibilities:**

- Monitors wheel rotation speed (RPM)
- Calculates linear speed (km/h)
- Accepts configuration for reporting frequency
- Forwards frequency configuration to CAN devices

**Topics Published:**

- `/rover/wheel_<position>/wheel_status` ([gateway_msgs/WheelStatus](messages.md#gateway_msgswheelstatus))

**Topics Subscribed:**

- `/rover/wheel_<position>/report_frequency_hz` ([gateway_msgs/ReportFrequency](messages.md#gateway_msgsreportfrequency))
- `/rover/mayor/can_status` ([gateway_msgs/CANStatus](messages.md#gateway_msgscanstatus))

**CAN Messages:**

- Receives wheel speed data
- Sends report frequency configuration

**Key Features:**

- **Configurable Reporting**: Adjustable frequency (1-100 Hz)
- **Input Validation**: Clamps frequency to valid range
- **CAN Status Awareness**: Only sends configuration when CAN enabled
- **Position-Specific CAN IDs**: Each wheel has unique CAN identifiers

**Parameters:**

- `position` (string): Wheel position
  - `"front_left"` - Front left wheel
  - `"front_right"` - Front right wheel
  - `"rear_left"` - Rear left wheel
  - `"rear_right"` - Rear right wheel

**Instances:**

| Node Name | Position | Location |
|-----------|----------|----------|
| `wheel_front_left` | `front_left` | Front left corner |
| `wheel_front_right` | `front_right` | Front right corner |
| `wheel_rear_left` | `rear_left` | Rear left corner |
| `wheel_rear_right` | `rear_right` | Rear right corner |

### Obstacle Detector Node

**Executable:** `obstacle_detector_node`

**Purpose:** Obstacle distance sensing

**Responsibilities:**

- Monitors 4 ultrasonic distance sensors per array
- Publishes distance readings in millimeters
- Provides separate monitoring for front and rear

**Topics Published:**

- `/rover/obstacle_detector_<position>/obstacle_distance` ([gateway_msgs/ObstacleDistance](messages.md#gateway_msgsobstacledistance))

**CAN Messages:**

- Receives distance readings (4 sensors per message)

**Key Features:**

- **Multi-Sensor Array**: 4 distance measurements per detector
- **Position-Specific CAN IDs**: Different IDs for front/rear
- **Millimeter Precision**: Distance values in millimeters

**Parameters:**

- `position` (string): Detector position
  - `"front"` - Front obstacle detector
  - `"rear"` - Rear obstacle detector

**Instances:**

| Node Name | Position | Location |
|-----------|----------|----------|
| `obstacle_detector_front` | `front` | Front of rover |
| `obstacle_detector_rear` | `rear` | Rear of rover |

## Node Communication

### CAN Bus Coordination

All nodes share a single CAN bus connection but use message filtering to receive only relevant messages:

```python
can_filters = [
    {"can_id": <node_specific_id>, "can_mask": 0x7FF}
]
```

**Threading Model:**

- Each node spawns a daemon CAN reader thread
- Reader threads process incoming messages continuously
- Main thread handles ROS2 callbacks and publishing

### ROS2 Communication Patterns

**Control Path:**

```
User → /rover/cmd_vel → controller_node → CAN Bus → Hardware
```

**Sensor Path:**

```
Hardware → CAN Bus → sensor_node → /rover/<node>/status → User
```

**Configuration Path:**

```
User → /rover/<node>/config → node → CAN Bus → Hardware
```

## Monitoring Nodes

### List Active Nodes

```bash
ros2 node list
```

Expected output:

```
/rover/battery_monitor_ad_system
/rover/battery_monitor_control_system
/rover/controller
/rover/mayor
/rover/obstacle_detector_front
/rover/obstacle_detector_rear
/rover/wheel_front_left
/rover/wheel_front_right
/rover/wheel_rear_left
/rover/wheel_rear_right
```

### Node Information

```bash
# Get detailed node info
ros2 node info /rover/controller

# List node's topics
ros2 node info /rover/wheel_front_left | grep -A 20 "Publishers"

# Check node parameters
ros2 param list /rover/battery_monitor_control_system
```

### Node Logging

```bash
# View node logs in real-time
ros2 run rqt_console rqt_console

# Or use command line
ros2 topic echo /rosout
```
