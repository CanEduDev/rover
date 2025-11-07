# Rover CAN Protocol

This section details all CAN messages in the Rover. This page contains a list of all messages with links to the details. Before diving in, read the general information below.

## General Information

**Default Bitrate**: 125 kbit/s

**Implementation**: Available on [GitHub](http://github.com/CanEduDev/rover) with DBC file for message decoding

**CAN ID Configuration**: All message CAN IDs can be configured offline and during runtime. Contact [support@canedudev.com](mailto:support@canedudev.com) for instructions

**Byte Order**: All multi-byte values are transmitted in little-endian format

- Example: `0x1234` is transmitted as byte 0 = `0x34`, byte 1 = `0x12`

**Version Tags**: Newer messages include a version tag (e.g. `v0.14.0`) indicating the minimum firmware version required to support that message. Ensure your firmware version meets or exceeds the specified version for the messages you intend to use.

**Documentation**: This document is updated regularly. Report errors or unclear sections to [support@canedudev.com](mailto:support@canedudev.com)

## List of Messages

| CAN ID | Message Name | DLC | Description |
| :---- | :---- | :---- | :---- |
| 0x100 | [Steering](control-messages.md#steering) | 5 | Set steering servo position |
| 0x101 | [Throttle](control-messages.md#throttle) | 5 | Set throttle value |
| 0x120 | [Light Array Front State](control-messages.md#light-array-front-state) | 4 | Set the front light array state |
| 0x121 | [Light Array Rear State](control-messages.md#light-array-rear-state) | 4 | Set the rear light array state |
| 0x122 | [Buzzer Sound](control-messages.md#buzzer-sound) | 6 | Play sounds through the buzzer |
| 0x200 | [Battery Cell Voltages](status-messages.md#battery-cell-voltages) | 7 | Report individual cell voltages |
| 0x201 | [Battery Regulated Output](status-messages.md#battery-regulated-output) | 8 | Report regulated power output voltage and current |
| 0x202 | [Battery Output](status-messages.md#battery-output) | 8 | Report battery output voltage and current |
| 0x203 | [Servo Voltage](status-messages.md#servo-voltage) | 2 | Report steering servo operating voltage |
| 0x204 | [Servo Current](status-messages.md#servo-current) | 2 | Report steering servo current usage |
| 0x205 | [Battery Voltage](status-messages.md#battery-voltage) | 2 | Report the battery voltage |
| 0x210 | [Front Left Wheel Speed](status-messages.md#front-left-wheel-speed) | 8 | Reports the front left wheel speed |
| 0x211 | [Front Right Wheel Speed](status-messages.md#front-right-wheel-speed) | 8 | Reports the front right wheel speed |
| 0x212 | [Rear Left Wheel Speed](status-messages.md#rear-left-wheel-speed) | 8 | Reports the rear left wheel speed |
| 0x213 | [Rear Right Wheel Speed](status-messages.md#rear-right-wheel-speed) | 8 | Reports the rear right wheel speed |
| 0x214 | [Obstacle Detector Front Distance](status-messages.md#obstacle-detector-front-distance) | 8 | Reports the front obstacle detector distances |
| 0x215 | [Obstacle Detector Rear Distance](status-messages.md#obstacle-detector-rear-distance) | 8 | Reports the rear obstacle detector distances |
| 0x311 | [Front Left Wheel Parameters](configuration-messages.md#front-left-wheel-parameters) | 8 | Configure wheel parameters for odometry |
| 0x312 | [Front Right Wheel Parameters](configuration-messages.md#front-right-wheel-parameters) | 8 | Configure wheel parameters for odometry |
| 0x313 | [Rear Left Wheel Parameters](configuration-messages.md#rear-left-wheel-parameters) | 8 | Configure wheel parameters for odometry |
| 0x314 | [Rear Right Wheel Parameters](configuration-messages.md#rear-right-wheel-parameters) | 8 | Configure wheel parameters for odometry |
| 0x300 | [Battery Jumper Config](configuration-messages.md#battery-jumper-config) | 1 | Set battery current measurement jumper config |
| 0x301 | [Battery Regulated Output Voltage](configuration-messages.md#battery-regulated-output-voltage) | 4 | Set the regulated power output voltage |
| 0x302 | [Battery Output ON/OFF](configuration-messages.md#battery-output-onoff) | 2 | Set the main power and regulated power states |
| 0x303 | [Battery Report Frequency](configuration-messages.md#battery-report-frequency) | 2 | Set battery report frequency |
| 0x304 | [Battery Low Voltage Cutoff](configuration-messages.md#battery-low-voltage-cutoff) | 8 | Configure low-voltage cutoff parameters |
| 0x305 | [Servo Set Voltage](configuration-messages.md#servo-set-voltage) | 2 | Set steering servo operating voltage |
| 0x306 | [Servo PWM Config](configuration-messages.md#servo-pwm-config) | 2 | Set servo PWM settings |
| 0x307 | [Servo Report Frequency](configuration-messages.md#servo-report-frequency) | 2 | Set servo report frequency |
| 0x308 | [Motor PWM Config](configuration-messages.md#motor-pwm-config) | 2 | Set motor controller PWM settings |
| 0x309 | [Servo Reverse Direction](configuration-messages.md#servo-reverse-direction) | 0 | Reverse steering servo direction |
| 0x30A | [Motor Reverse Direction](configuration-messages.md#motor-reverse-direction) | 0 | Reverse motor throttle direction |
| 0x30B | [Servo Failsafe](configuration-messages.md#servo-failsafe) | 5 | Configure steering servo failsafe settings |
| 0x30C | [Motor Failsafe](configuration-messages.md#motor-failsafe) | 5 | Configure motor failsafe settings |
| 0x30D | [Steering Subtrim](configuration-messages.md#steering-subtrim) | 2 | Configure steering servo subtrim |
| 0x30E | [Throttle Subtrim](configuration-messages.md#throttle-subtrim) | 2 | Configure throttle subtrim |
| 0x30F | [Battery main power overcurrent threshold](configuration-messages.md#battery-main-power-overcurrent-threshold) | 4 | Set overcurrent threshold for main power output |
| 0x310 | [Battery regulated output overcurrent threshold](configuration-messages.md#battery-regulated-output-overcurrent-threshold) | 4 | Set overcurrent threshold for regulated power output |
| 0x500 | [AD Battery Cell Voltages](status-messages.md#ad-battery-cell-voltages) | 7 | Report individual cell voltages |
| 0x501 | [AD Battery Regulated Output](status-messages.md#ad-battery-regulated-output) | 8 | Report regulated power output voltage and current |
| 0x502 | [AD Battery Output](status-messages.md#ad-battery-output) | 8 | Report battery output voltage and current |
| 0x600 | [AD Battery Jumper Config](configuration-messages.md#ad-battery-jumper-config) | 1 | Set battery current measurement jumper config |
| 0x601 | [AD Battery Regulated Output Voltage](configuration-messages.md#ad-battery-regulated-output-voltage) | 4 | Set the regulated power output voltage |
| 0x602 | [AD Battery Output ON/OFF](configuration-messages.md#ad-battery-output-onoff) | 2 | Set the main power and regulated power states |
| 0x603 | [AD Battery Report Frequency](configuration-messages.md#ad-battery-report-frequency) | 2 | Set battery report frequency |
| 0x604 | [AD Battery Low Voltage Cutoff](configuration-messages.md#ad-battery-low-voltage-cutoff) | 8 | Configure low-voltage cutoff parameters |
| 0x60F | [AD Battery main power overcurrent threshold](configuration-messages.md#ad-battery-main-power-overcurrent-threshold) | 4 | Set overcurrent threshold for main power output |
| 0x610 | [AD Battery regulated output overcurrent threshold](configuration-messages.md#ad-battery-regulated-output-overcurrent-threshold) | 4 | Set overcurrent threshold for regulated power output |
