# Messages and Topics

The Rover gateway bridges CAN bus messages with ROS2 topics, using both standard ROS2 messages (like `geometry_msgs/Twist`) and custom messages defined in the `gateway_msgs` package. All topics use the `/rover/` namespace prefix.

## Messages

The gateway uses both standard and custom ROS2 messages. All messages in the `gateway_msgs/` package are custom.

#### geometry_msgs/Twist

Vehicle control message containing linear and angular velocity components.

**Message Definition:**
```
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```

**Field Usage:**

- `linear.x` (float64): Forward/backward throttle command
  - Range: -1.0 to 1.0
  - `-1.0` = full reverse, `0.0` = neutral, `1.0` = full forward
  - Internally converted to PWM pulse width (1000-2000µs, 1500µs neutral)
- `angular.z` (float64): Steering angle in radians
  - Range: ±π/4 radians (±45 degrees)
  - Positive = right turn, negative = left turn
- Other fields: Unused by Rover

**Safety Notes:**

- Physical radio overrides all `/rover/cmd_vel` commands when active
- Direction changes require stopping at neutral for 2 seconds
- Values are validated and clamped to safe ranges

---

#### gateway_msgs/BatteryStatus

Battery monitoring with cell voltages, output power, and regulated power.

**Message Definition:**
```
# Cell voltages in millivolts
uint16[] cell_voltages_mv

# Battery output voltage in millivolts
uint32 output_voltage_mv

# Battery output current in milliamps
uint32 output_current_ma

# Regulated output voltage in millivolts
uint32 regulated_voltage_mv

# Regulated output current in milliamps
uint32 regulated_current_ma
```

**Field Details:**

- `cell_voltages_mv`: Array of 6 values for individual cells (LiPo battery)
- `output_voltage_mv`: Raw battery pack output voltage
- `output_current_ma`: Current being drawn from battery
- `regulated_voltage_mv`: Voltage after regulation (5V/3.3V rails)
- `regulated_current_ma`: Current on regulated outputs

**CAN Mapping:**

- Cell voltages split across two CAN messages (3 cells each)
- Published only when both messages received

---

#### gateway_msgs/WheelStatus

Wheel speed and RPM information.

**Message Definition:**
```
# Wheel RPM (revolutions per minute)
float32 rpm

# Wheel speed in kilometers per hour
float32 speed_kph
```

**Field Details:**

- `rpm`: Raw wheel rotation speed
- `speed_kph`: Calculated linear speed (based on wheel diameter)

---

#### gateway_msgs/ObstacleDistance

Distance measurements from ultrasonic sensors.

**Message Definition:**
```
# Distance readings in millimeters for each sensor
# Array of 4 distance values
uint16[] distances_mm
```

**Field Details:**

- `distances_mm`: Array of exactly 4 measurements in millimeters
- Each element corresponds to a sensor position (left to right)
- Value of 0 may indicate sensor error or out-of-range

---

#### gateway_msgs/CANStatus

CAN bus communication status.

**Message Definition:**
```
# Whether CAN communication is enabled
bool can_enabled

# Timestamp when the status was last updated
uint64 timestamp
```

**Field Details:**

- `can_enabled`: True when CAN active, False when disabled
- `timestamp`: Microsecond timestamp of status update

**Usage:**

- CAN is disabled during FREEZE action mode or LISTEN_ONLY/SILENT communication modes
- Used by firmware update utility to pause operation of all modules

---

#### gateway_msgs/ReportFrequency

Configuration for sensor reporting frequency.

**Message Definition:**
```
# Frequency in Hz for sensor reporting
uint32 frequency_hz
```

**Field Details:**

- `frequency_hz`: Desired reporting frequency
- Valid range: 1-100 Hz
- Out-of-range values are clamped

## Topics

Topics follow the pattern:
```
/rover/<node_name>/<message_type>
```

For multi-instance components (wheels, batteries, obstacle detectors):
```
/rover/<component>_<position>/<message_type>
```

### Control Topics

#### /rover/cmd_vel

**Type:** `geometry_msgs/Twist`

**Direction:** Subscribed by gateway

**QoS:** `RELIABLE`

**Purpose:** Control commands for throttle and steering

**Example:**
```bash
# Drive forward at half speed with slight right turn
ros2 topic pub /rover/cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.5, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.349}"
```

**Python Example:**
```python
from geometry_msgs.msg import Twist
import math

cmd = Twist()
cmd.linear.x = 0.5    # 50% forward throttle
cmd.angular.z = math.radians(20)  # 20 degrees right
publisher.publish(cmd)
```

---

### Monitoring Topics

/// warning | Read-only topics
All topics in this section are published by the gateway. Do not publish to these topics - they are for monitoring only.
///

#### /rover/radio/cmd_vel

**Type:** `geometry_msgs/Twist`

**Direction:** Published by gateway

**QoS:** `BEST_EFFORT`

**Purpose:** Radio control commands (published when physical radio active)

#### Battery Monitor Topics

**Instances:**

- `battery_monitor_control_system` - Control system battery
- `battery_monitor_ad_system` - Autonomous driving system battery

#### /rover/battery_monitor_*/cell_voltages

**Type:** `gateway_msgs/BatteryStatus`

**QoS:** `BEST_EFFORT`

**Purpose:** Individual cell voltages

**Example:**
```bash
ros2 topic echo /rover/battery_monitor_control_system/cell_voltages
```

#### /rover/battery_monitor_*/output

**Type:** `gateway_msgs/BatteryStatus`

**QoS:** `BEST_EFFORT`

**Purpose:** Battery output voltage and current

#### /rover/battery_monitor_*/regulated

**Type:** `gateway_msgs/BatteryStatus`

**QoS:** `BEST_EFFORT`

**Purpose:** Regulated output voltage and current

#### Wheel Topics

**Instances:**

- `wheel_front_left`
- `wheel_front_right`
- `wheel_rear_left`
- `wheel_rear_right`

#### /rover/wheel_*/wheel_status

**Type:** `gateway_msgs/WheelStatus`

**QoS:** `BEST_EFFORT`

**Purpose:** Wheel speed and RPM

**Example:**
```bash
# Monitor front left wheel
ros2 topic echo /rover/wheel_front_left/wheel_status
```

#### Obstacle Detector Topics

**Instances:**

- `obstacle_detector_front`
- `obstacle_detector_rear`

#### /rover/obstacle_detector_*/obstacle_distance

**Type:** `gateway_msgs/ObstacleDistance`

**QoS:** `BEST_EFFORT`

**Purpose:** Distance readings from ultrasonic sensors

**Example:**
```bash
ros2 topic echo /rover/obstacle_detector_front/obstacle_distance
```

#### /rover/mayor/can_status

**Type:** `gateway_msgs/CANStatus`

**QoS:** `RELIABLE`

**Purpose:** CAN bus status information

---

### Configuration Topics

#### /rover/wheel_*/report_frequency_hz

**Type:** `gateway_msgs/ReportFrequency`

**QoS:** `RELIABLE`

**Direction:** Subscribed by gateway

**Purpose:** Configure wheel reporting frequency

**Instances:**

- `wheel_front_left`
- `wheel_front_right`
- `wheel_rear_left`
- `wheel_rear_right`

**Example:**

```bash
# Set to 50 Hz
ros2 topic pub /rover/wheel_front_left/report_frequency_hz \
  gateway_msgs/ReportFrequency "{frequency_hz: 50}"
```

---

## Topic Discovery

List all active Rover topics:
```bash
ros2 topic list | grep /rover
```

Monitor a specific topic:
```bash
ros2 topic echo /rover/wheel_front_left/wheel_status
```

Check topic info:
```bash
ros2 topic info /rover/cmd_vel
```

Get message type details:
```bash
ros2 interface show geometry_msgs/Twist
ros2 interface show gateway_msgs/BatteryStatus
```

Measure topic frequency:
```bash
ros2 topic hz /rover/wheel_front_left/wheel_status
```
