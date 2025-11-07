# Status Messages


These messages are available to read the status of the Rover.

/// note
All status messages are read-only.
///

### Battery Cell Voltages

**CAN ID**: 0x200 | **DLC**: 7 bytes

Report the individual cell voltages of the connected battery. This message is sent twice: once with values for cells 1-3 and once for cells 4-6.

**Cell Group Selection (Byte 0):**

- `0` = Cell group 0 (Cells 1, 2, 3)
- `1` = Cell group 1 (Cells 4, 5, 6)

#### Cell Group 0 (Cells 1-3)

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 1-2 | Voltage in mV | Cell 1 voltage. Unsigned 16-bit integer. |
| 3-4 | Voltage in mV | Cell 2 voltage. Unsigned 16-bit integer. |
| 5-6 | Voltage in mV | Cell 3 voltage. Unsigned 16-bit integer. |

#### Cell Group 1 (Cells 4-6)

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 1-2 | Voltage in mV | Cell 4 voltage. Unsigned 16-bit integer. |
| 3-4 | Voltage in mV | Cell 5 voltage. Unsigned 16-bit integer. |
| 5-6 | Voltage in mV | Cell 6 voltage. Unsigned 16-bit integer. |

---

### Battery Regulated Output

**CAN ID**: 0x201 | **DLC**: 8 bytes

Report the voltage and current of the regulated power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Voltage in mV | Output voltage in mV. Unsigned 32-bit integer. |
| 4-7 | Current in mA | Output current in mA. Unsigned 32-bit integer. |

---

### Battery Output

**CAN ID**: 0x202 | **DLC**: 8 bytes

Report the voltage and current of the main power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Voltage in mV | Output voltage in mV. Unsigned 32-bit integer. |
| 4-7 | Current in mA | Output current in mA. Unsigned 32-bit integer. |

---

### Servo Voltage

**CAN ID**: 0x203 | **DLC**: 2 bytes

Report the servo operating voltage.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | Voltage in mV | Servo voltage in mV. Unsigned 16-bit integer. |

---

### Servo Current

**CAN ID**: 0x204 | **DLC**: 2 bytes

Report the servo current usage.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | Current in mA | Servo current usage in mA. Unsigned 16-bit integer. |

---

### Battery Voltage

**CAN ID**: 0x205 | **DLC**: 2 bytes

Report the battery voltage as seen from the steering servo.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | Voltage in mV | Battery voltage in mV. Unsigned 16-bit integer. |

---

### Front Left Wheel Speed

**CAN ID**: 0x210 | **DLC**: 8 bytes

Reports the front left wheel speed.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Varies | Wheel speed in RPM. Signed 32-bit integer. |
| 4-7 | Varies | Wheel speed in km/h. Signed 32-bit integer. |

---

### Front Right Wheel Speed

**CAN ID**: 0x211 | **DLC**: 8 bytes

Reports the front right wheel speed.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Varies | Wheel speed in RPM. Signed 32-bit integer. |
| 4-7 | Varies | Wheel speed in km/h. Signed 32-bit integer. |

---

### Rear Left Wheel Speed

**CAN ID**: 0x212 | **DLC**: 8 bytes

Reports the rear left wheel speed.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Varies | Wheel speed in RPM. Signed 32-bit integer. |
| 4-7 | Varies | Wheel speed in km/h. Signed 32-bit integer. |

---

### Rear Right Wheel Speed

**CAN ID**: 0x213 | **DLC**: 8 bytes

Reports the rear right wheel speed.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Varies | Wheel speed in RPM. Signed 32-bit integer. |
| 4-7 | Varies | Wheel speed in km/h. Signed 32-bit integer. |

---

### Obstacle Detector Front Distance

**CAN ID**: 0x214 | **DLC**: 8 bytes

Reports the front obstacle detector distances.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | 20-5000 | Distance measured by the left sensor in mm. |
| 2-3 | 20-5000 | Distance measured by the mid-left sensor in mm. |
| 4-5 | 20-5000 | Distance measured by the mid-right sensor in mm. |
| 6-7 | 20-5000 | Distance measured by the right sensor in mm. |

---

### Obstacle Detector Rear Distance

**CAN ID**: 0x215 | **DLC**: 8 bytes

Reports the rear obstacle detector distances.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | 20-5000 | Distance measured by the left sensor in mm. |
| 2-3 | 20-5000 | Distance measured by the mid-left sensor in mm. |
| 4-5 | 20-5000 | Distance measured by the mid-right sensor in mm. |
| 6-7 | 20-5000 | Distance measured by the right sensor in mm. |

---

### AD Battery Cell Voltages

**CAN ID**: 0x500 | **DLC**: 7 bytes

Report the individual cell voltages of the connected battery. This message is sent twice: once with values for cells 1-3 and once for cells 4-6.

**Cell Group Selection (Byte 0):**

- `0` = Cell group 0 (Cells 1, 2, 3)
- `1` = Cell group 1 (Cells 4, 5, 6)

#### Cell Group 0 (Cells 1-3)

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 1-2 | Voltage in mV | Cell 1 voltage. Unsigned 16-bit integer. |
| 3-4 | Voltage in mV | Cell 2 voltage. Unsigned 16-bit integer. |
| 5-6 | Voltage in mV | Cell 3 voltage. Unsigned 16-bit integer. |

#### Cell Group 1 (Cells 4-6)

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 1-2 | Voltage in mV | Cell 4 voltage. Unsigned 16-bit integer. |
| 3-4 | Voltage in mV | Cell 5 voltage. Unsigned 16-bit integer. |
| 5-6 | Voltage in mV | Cell 6 voltage. Unsigned 16-bit integer. |

---

### AD Battery Regulated Output

**CAN ID**: 0x501 | **DLC**: 8 bytes

Report the voltage and current of the regulated power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Voltage in mV | Output voltage in mV. Unsigned 32-bit integer. |
| 4-7 | Current in mA | Output current in mA. Unsigned 32-bit integer. |

---

### AD Battery Output

**CAN ID**: 0x502 | **DLC**: 8 bytes

Report the voltage and current of the main power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Voltage in mV | Output voltage in mV. Unsigned 32-bit integer. |
| 4-7 | Current in mA | Output current in mA. Unsigned 32-bit integer. |

---

