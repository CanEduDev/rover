# Control Messages

These messages are used to control various parts of the Rover. Message availability is dependent on which addon the Rover is equipped with. For example, Rovers with the base configuration are not equipped with the Light Array module.

### Steering

**CAN ID**: 0x100 | **DLC**: 5 bytes

Control the steering servo position. This message should be sent periodically with a frequency of at least 20Hz (every 50ms).

**Mode Selection (Byte 0):**

- `0` = Pulse-width steering mode
- `1` = Angle steering mode

#### Pulse-width Steering Mode

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 1-2 | 1000 - 2000 | Sets the steering servo position using pulse width in microseconds. Unsigned 16-bit integer. |
| 3-4 | 0 | Ignored. |

#### Angle Steering Mode

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 1-4 | -45 to 45 | Sets the steering servo position using an angle value in degrees. 32-bit float. |

---

### Throttle

**CAN ID**: 0x101 | **DLC**: 5 bytes

Set the throttle value at the ESC. This message should be sent periodically with a frequency of at least 20Hz (every 50ms). 


Throttle values:

- 1000 µs: full brake/full reverse
- 1500 µs: neutral position
- 2000 µs: full throttle

To reverse, stop the rover, set to neutral, wait 250ms, then send reverse pulse. 

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0 | Set to 0 for pulse-width mode. |
| 1-2 | 1000 - 2000 | Throttle pulse-width in microseconds. Unsigned 16-bit integer.|
| 3-4 | 0 | Ignored. |

---

### Light Array Front State

**CAN ID**: 0x120 | **DLC**: 4 bytes

Control the state of the front light array.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0 or 1 | Left light, 0 is OFF, 1 is ON |
| 1 | 0 or 1 | Mid left light, 0 is OFF, 1 is ON |
| 2 | 0 or 1 | Mid right light, 0 is OFF, 1 is ON |
| 3 | 0 or 1 | Right light, 0 is OFF, 1 is ON |

---

### Light Array Rear State

**CAN ID**: 0x121 | **DLC**: 4 bytes

Control the state of the rear light array.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0 or 1 | Left light, 0 is OFF, 1 is ON |
| 1 | 0 or 1 | Mid left light, 0 is OFF, 1 is ON |
| 2 | 0 or 1 | Mid right light, 0 is OFF, 1 is ON |
| 3 | 0 or 1 | Right light, 0 is OFF, 1 is ON |

---

### Buzzer Sound

`v0.14.0` | **CAN ID**: 0x122 | **DLC**: 6 bytes

Play sounds using the buzzer. All sounds are generated as PWM signals of a specified frequency. The volume corresponds to the pulse-width of the PWM signal.

/// note
At most 8 sounds can be enqueued. Overflowing the queue will cause sounds to be dropped.
/// 

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | Frequency in Hz | Frequency of the sound to be played. A value of 0 is considered a silent sound. Unsigned 16-bit integer. |
| 2-3 | Duration in ms | How long the sound should be played. When the duration has passed, the buzzer is muted. A value of 0 is ignored. |
| 4-5 | Pulse width in µs | Sound volume. A value of 0 means the buzzer should be muted. Typical values are 0-100. The firmware will limit the volume to 90% duty cycle. |

---

