# Servo board testing frames

from canlib import Frame
import ck

# Assign envelope 0x200 to folder 2
# Sensor power transmit
assign_sensor_power_tx = Frame(id_=0, dlc=8, data=[ck.servo_id, 2, 0, 2, 0, 0, 2, 0x3])

# Assign envelope 0x201 to folder 3
# Servo current transmit
assign_servo_current_tx = Frame(id_=0, dlc=8, data=[ck.servo_id, 2, 1, 2, 0, 0, 3, 0x3])

# Assign envelope 0x202 to folder 4
# Battery voltage transmit
assign_battery_voltage_tx = Frame(
    id_=0, dlc=8, data=[ck.servo_id, 2, 2, 2, 0, 0, 4, 0x3]
)

# Assign envelope 0x203 to folder 5
# Servo voltage transmit
assign_servo_voltage_tx = Frame(id_=0, dlc=8, data=[ck.servo_id, 2, 3, 2, 0, 0, 5, 0x3])

# Assign envelope 0x204 to folder 6
# H-bridge current transmit
assign_h_bridge_current_tx = Frame(
    id_=0, dlc=8, data=[ck.servo_id, 2, 4, 2, 0, 0, 6, 0x3]
)

# Assign envelope 0x205 to folder 7
# Set servo voltage receive
assign_servo_voltage_rx = Frame(id_=0, dlc=8, data=[ck.servo_id, 2, 5, 2, 0, 0, 7, 0x3])

# Assign envelope 0x206 to folder 8
# PWM frequency config receive
assign_pwm_config_rx = Frame(id_=0, dlc=8, data=[ck.servo_id, 2, 6, 2, 0, 0, 8, 0x3])

# Assign envelope 0x207 to folder 9
# Steering signal receive (pulse or angle)
assign_steering_rx = Frame(id_=0, dlc=8, data=[ck.servo_id, 2, 7, 2, 0, 0, 9, 0x3])

# Assign envelope 0x208 to folder 10
# Steering trim signal receive (pulse)
assign_steering_trim_rx = Frame(
    id_=0, dlc=8, data=[ck.servo_id, 2, 8, 2, 0, 0, 10, 0x3]
)

# Assign envelope 0x209 to folder 11
# CAN report frequency receive
assign_report_freq_rx = Frame(id_=0, dlc=8, data=[ck.servo_id, 2, 9, 2, 0, 0, 11, 0x3])

# Set servo votlage to 7400 mV
set_servo_voltage_7400mv = Frame(id_=0x205, dlc=2, data=[0xE8, 0x1C])

# Set PWM frequency to 333 Hz
set_pwm_conf_333hz = Frame(id_=0x206, dlc=2, data=[0x4D, 0x1])

# Set steering PWM pulse to 500 µs
set_steer_pulse_1000 = Frame(id_=0x207, dlc=3, data=[0, 0xE8, 0x3])

# Set steering PWM pulse to 2500 µs
set_steer_pulse_2000 = Frame(id_=0x207, dlc=3, data=[0, 0xD0, 0x7])

# Set steering angle to 90 degrees
set_steer_angle_45 = Frame(id_=0x207, dlc=3, data=[1, 0x2D, 0])

# Set steering angle to -90 degrees
set_steer_angle_minus_45 = Frame(id_=0x207, dlc=3, data=[1, 0xD3, 0xFF])

# Set steering trim PWM pulse to 200 µs
set_steer_trim_pulse_200 = Frame(id_=0x208, dlc=3, data=[0, 0xC8, 0x0])

# Set steering trim PWM pulse to -200 µs
set_steer_trim_pulse_minus_200 = Frame(id_=0x208, dlc=3, data=[0, 0x38, 0xFF])

# Set steering trim PWM angle to 15 degrees
set_steer_trim_angle_15 = Frame(id_=0x208, dlc=3, data=[1, 0x0F, 0x0])

# Set steering trim PWM pulse to -15 degrees
set_steer_trim_angle_minus_15 = Frame(id_=0x208, dlc=3, data=[1, 0xF1, 0xFF])

# Set steering trim PWM pulse to 0
set_steer_trim_pulse_0 = Frame(id_=0x208, dlc=3, data=[0, 0, 0])

# Set measure frequency to 500 ms. Ignore report frequency.
set_measure_freq_500ms = Frame(id_=0x209, dlc=4, data=[0xF4, 0x1, 0, 0])

# Set report frequency to 1000 ms. Ignore measure frequency.
set_report_freq_1000ms = Frame(id_=0x209, dlc=4, data=[0, 0, 0xE8, 0x3])
