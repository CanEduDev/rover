# Servo board testing frames
import struct

import can

from . import City, Envelope

FAILSAFE_OFF = 0
FAILSAFE_ON = 1
FAILSAFE_KEEP_CURRENT = 0xFF


def set_servo_voltage_frame(voltage_mv):
    data = list(voltage_mv.to_bytes(2, "little"))
    return can.Message(
        arbitration_id=Envelope.SERVO_SET_VOLTAGE,
        dlc=2,
        data=data,
        is_extended_id=False,
    )


def set_pwm_frequency_frame(frequency_hz):
    data = list(frequency_hz.to_bytes(2, "little"))
    return can.Message(
        arbitration_id=Envelope.SERVO_PWM_CONFIG, dlc=2, data=data, is_extended_id=False
    )


def set_steering_pulse_frame(pulse_mus):
    data = [0] + list(pulse_mus.to_bytes(2, "little", signed=True))
    return can.Message(
        arbitration_id=Envelope.STEERING, dlc=5, data=data, is_extended_id=False
    )


def set_steering_angle_frame(angle_deg):
    data = [1] + list(struct.pack("f", angle_deg))
    return can.Message(
        arbitration_id=Envelope.STEERING, dlc=5, data=data, is_extended_id=False
    )


def set_throttle_pulse_frame(pulse_mus):
    data = [0] + list(pulse_mus.to_bytes(2, "little", signed=True))
    return can.Message(
        arbitration_id=Envelope.THROTTLE, dlc=5, data=data, is_extended_id=False
    )


def set_report_period_frame(time_ms):
    data = list(time_ms.to_bytes(2, "little"))
    return can.Message(
        arbitration_id=Envelope.SERVO_REPORT_FREQUENCY,
        dlc=2,
        data=data,
        is_extended_id=False,
    )


def set_reverse_direction():
    return can.Message(
        arbitration_id=Envelope.SERVO_REVERSE_DIRECTION,
        dlc=0,
        data=[],
        is_extended_id=False,
    )


def set_failsafe(on, city=City.SERVO, timeout_ms=100, pulse_mus=1500):
    if city == City.MOTOR:
        envelope = Envelope.MOTOR_FAILSAFE
    else:
        envelope = Envelope.SERVO_FAILSAFE

    data = (
        [on]
        + list(timeout_ms.to_bytes(2, "little"))
        + list(pulse_mus.to_bytes(2, "little"))
    )

    return can.Message(arbitration_id=envelope, dlc=5, data=data, is_extended_id=False)
