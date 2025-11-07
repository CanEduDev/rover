# Configuration Messages


The Rover comes with sane default settings, but if needed the settings can be changed using the configuration messages below.

/// note
These settings are set during runtime and will not persist between reboots of the Rover.
Send the message periodically in your application to work around this limitation.
///

### Battery Jumper Config

**CAN ID**: 0x300 | **DLC**: 1 byte

Set the power board's current jumper configuration.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0-3 | X11 and X12 headers jumper configuration:<br>• 0 = X11 OFF, X12 OFF<br>• 1 = X11 ON, X12 OFF<br>• 2 = X11 OFF, X12 ON<br>• 3 = X11 ON, X12 ON |

---

### Battery Regulated Output Voltage

**CAN ID**: 0x301 | **DLC**: 4 bytes

Set the target output voltage for the regulated power output. The power board will try to get as close as possible to the given voltage.

**WARNING**: Measure the voltage and verify it's correct before connecting anything to the output. Otherwise you may damage the connected components.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Voltage in mV | Regulated output voltage in mV. Unsigned 32-bit integer.<br>Voltage range depends on JP1 jumper configuration:<br>• JP1 = 5V: range is 3-6V<br>• JP1 = 12V: range is 6-16V |

---

### Battery Output ON/OFF

**CAN ID**: 0x302 | **DLC**: 2 bytes

Set the battery output state for the main power and the regulated power outputs.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0 or 1 | Main power output state. 0 is OFF, 1 is ON. Note that turning the main power off will also turn the regulated power off. |
| 1 | 0 or 1 | Regulated power output state. 0 is OFF, 1 is ON. |

---

### Battery Report Frequency

**CAN ID**: 0x303 | **DLC**: 2 bytes

Set the battery board report frequency.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | ms | Reporting period in ms, i.e. how often to send measurements over CAN. Unsigned 16-bit integer. Setting to 0 ignores this value. |

---

### Battery Low Voltage Cutoff

**CAN ID**: 0x304 | **DLC**: 8 bytes

Configure the low-voltage cutoff parameters. Two cutoff voltages are configured: one for low load and one for high load conditions. When any connected cell is below the cutoff voltage, main power is turned off to protect the battery.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | Voltage in mV | Cutoff voltage for the low load condition. Unsigned 16-bit integer. |
| 2-3 | Voltage in mV | Cutoff voltage for the high load condition. Unsigned 16-bit integer. This value must be lower than the low load condition cutoff voltage. |
| 4-7 | Current in mA | High load threshold current. If the current draw is higher than this value, the power board will use the high load cutoff voltage. |

---

### Servo Set Voltage

**CAN ID**: 0x305 | **DLC**: 2 bytes

Set the servo operating voltage. The servo board will try to get as close as possible to the given voltage.

**WARNING**: Check the servo voltage ratings before setting this value, as setting too high a value might damage the servo.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | Voltage in mV | Servo voltage in mV. Unsigned 16-bit integer. |

---

### Servo PWM Config

**CAN ID**: 0x306 | **DLC**: 2 bytes

Set the steering servo PWM parameters.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | 1 - 333 | PWM frequency in Hz. Unsigned 16-bit integer. Setting to 0 ignores this value. |

---

### Servo Report Frequency

**CAN ID**: 0x307 | **DLC**: 2 bytes

Set the servo board report frequency.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | ms | Reporting period in ms, i.e. how often to send measurements over CAN. Unsigned 16-bit integer. Setting to 0 ignores this value. |

---

### Motor PWM Config

**CAN ID**: 0x308 | **DLC**: 2 bytes

Set the ESC PWM parameters.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | 1 - 333 | PWM frequency in Hz. Unsigned 16-bit integer. Setting to 0 ignores this value. |

---

### Servo Reverse Direction

**CAN ID**: 0x309 | **DLC**: 0 bytes

Reverse the steering servo control direction.

---

### Motor Reverse Direction

**CAN ID**: 0x30A | **DLC**: 0 bytes

Reverse the motor control direction.

---

### Servo Failsafe

**CAN ID**: 0x30B | **DLC**: 5 bytes

Configure the steering servo failsafe mechanism. This feature takes over control of the steering servo when the module does not receive any steering messages within the specified timeout period.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0 or 1 | Failsafe state: 0 = off, 1 = on |
| 1-2 | ms | Failsafe timeout period (how many ms without steering messages until failsafe triggers). Defaults to 100ms. |
| 3-4 | 1000-2000 | Pulse width to set when failsafe triggers. Defaults to 1500µs (neutral position). |

---

### Motor Failsafe

**CAN ID**: 0x30C | **DLC**: 5 bytes

Configure the motor control failsafe mechanism. This feature takes over control of the motor when the module does not receive any throttle messages within the specified timeout period.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0 or 1 | Failsafe state: 0 = off, 1 = on |
| 1-2 | ms | Failsafe timeout period (how many ms without throttle messages until failsafe triggers). Defaults to 100ms. |
| 3-4 | 1000-2000 | Pulse width to set when failsafe triggers. Defaults to 1500µs (neutral position). |

---

### Steering Subtrim

**CAN ID**: 0x30D | **DLC**: 2 bytes

Set the subtrim value (permanent offset) of the steering servo.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | -500 to 500 | Steering trim pulse-width in microseconds. Signed 16-bit integer. |

---

### Throttle Subtrim

**CAN ID**: 0x30E | **DLC**: 2 bytes

Set the subtrim value (permanent offset) of the throttle.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | -500 to 500 | Throttle trim pulse-width in microseconds. Signed 16-bit integer. |

---

### Battery Main Power Overcurrent Threshold

**CAN ID**: 0x30F | **DLC**: 4 bytes

Set the overcurrent threshold for the main power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Current in mA | Overcurrent threshold. Unsigned 32-bit integer. This value determines the maximum current draw allowed before the main power output is turned off to protect the system. |

---

### Battery Regulated Output Overcurrent Threshold

**CAN ID**: 0x310 | **DLC**: 4 bytes

Set the overcurrent threshold for the regulated power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Current in mA | Overcurrent threshold. Unsigned 32-bit integer. This value determines the maximum current draw allowed before the regulated power output is turned off to protect the system. |

---

### Front Left Wheel Parameters

**CAN ID**: 0x311 | **DLC**: 8 bytes

Set the wheel parameters for more accurate odometry.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Cog count | Number of cogs in the wheel speed module. Unsigned 32-bit integer. |
| 4-7 | Wheel diameter in m | Wheel outer diameter measured in meters. 32-bit float. |

---

### Front Right Wheel Parameters

**CAN ID**: 0x312 | **DLC**: 8 bytes

Set the wheel parameters for more accurate odometry.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Cog count | Number of cogs in the wheel speed module. Unsigned 32-bit integer. |
| 4-7 | Wheel diameter in m | Wheel outer diameter measured in meters. 32-bit float. |

---

### Rear Left Wheel Parameters

**CAN ID**: 0x313 | **DLC**: 8 bytes

Set the wheel parameters for more accurate odometry.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Cog count | Number of cogs in the wheel speed module. Unsigned 32-bit integer. |
| 4-7 | Wheel diameter in m | Wheel outer diameter measured in meters. 32-bit float. |

---

### Rear Right Wheel Parameters

**CAN ID**: 0x314 | **DLC**: 8 bytes

Set the wheel parameters for more accurate odometry.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Cog count | Number of cogs in the wheel speed module. Unsigned 32-bit integer. |
| 4-7 | Wheel diameter in m | Wheel outer diameter measured in meters. 32-bit float. |

---

### AD Battery Jumper Config

**CAN ID**: 0x600 | **DLC**: 1 byte

Set the AD power board's current jumper configuration.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0-3 | X11 and X12 headers jumper configuration:<br>• 0 = X11 OFF, X12 OFF<br>• 1 = X11 ON, X12 OFF<br>• 2 = X11 OFF, X12 ON<br>• 3 = X11 ON, X12 ON |

---

### AD Battery Regulated Output Voltage

**CAN ID**: 0x601 | **DLC**: 4 bytes

Set the target output voltage for the regulated power output. The power board will try to get as close as possible to the given voltage.

**WARNING**: Measure the voltage and verify it's correct before connecting anything to the output. Otherwise you may damage the connected components.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Voltage in mV | Regulated output voltage in mV. Unsigned 32-bit integer.<br>Voltage range depends on JP1 jumper configuration:<br>• JP1 = 5V: range is 3-6V<br>• JP1 = 12V: range is 6-16V |

---

### AD Battery Output ON/OFF

**CAN ID**: 0x602 | **DLC**: 2 bytes

Set the AD battery output state for the main power and the regulated power outputs.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0 | 0 or 1 | Main power output state. 0 is OFF, 1 is ON. Note that turning the main power off will also turn the regulated power off. |
| 1 | 0 or 1 | Regulated power output state. 0 is OFF, 1 is ON. |

---

### AD Battery Report Frequency

**CAN ID**: 0x603 | **DLC**: 2 bytes

Set the AD battery board report frequency.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | ms | Reporting period in ms, i.e. how often to send measurements over CAN. Unsigned 16-bit integer. Setting to 0 ignores this value. |

---

### AD Battery Low Voltage Cutoff

**CAN ID**: 0x604 | **DLC**: 8 bytes

Configure the low-voltage cutoff parameters for the AD battery. Two cutoff voltages are configured: one for low load and one for high load conditions. When any connected cell is below the cutoff voltage, main power is turned off to protect the battery.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-1 | Voltage in mV | Cutoff voltage for the low load condition. Unsigned 16-bit integer. |
| 2-3 | Voltage in mV | Cutoff voltage for the high load condition. Unsigned 16-bit integer. This value must be lower than the low load condition cutoff voltage. |
| 4-7 | Current in mA | High load threshold current. If the current draw is higher than this value, the power board will use the high load cutoff voltage. |

---

### AD Battery Main Power Overcurrent Threshold

**CAN ID**: 0x60F | **DLC**: 4 bytes

Set the overcurrent threshold for the AD battery main power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Current in mA | Overcurrent threshold. Unsigned 32-bit integer. This value determines the maximum current draw allowed before the main power output is turned off to protect the system. |

---

### AD Battery Regulated Output Overcurrent Threshold

**CAN ID**: 0x610 | **DLC**: 4 bytes

Set the overcurrent threshold for the AD battery regulated power output.

| Byte number | Values | Description |
| :---- | :---- | :---- |
| 0-3 | Current in mA | Overcurrent threshold. Unsigned 32-bit integer. This value determines the maximum current draw allowed before the regulated power output is turned off to protect the system. |
