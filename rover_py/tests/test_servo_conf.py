import argparse
import time

from rover import City, Envelope, rover, servo
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser(description="Test servo configuration")
    add_can_args(parser)
    args = parser.parse_args()

    with create_bus_from_args(args) as bus:
        # Set servo settings
        bus.send(servo.set_failsafe(servo.FAILSAFE_OFF))
        bus.send(servo.set_servo_voltage_frame(7400))
        bus.send(servo.set_pwm_frequency_frame(333))

        # Disable SBUS receiver city's communication
        bus.send(
            rover.set_comm_mode(city=City.SBUS_RECEIVER, mode=rover.CommMode.SILENT)
        )

        bus.set_filters([{"can_id": Envelope.SERVO_VOLTAGE, "can_mask": (1 << 11) - 1}])

        # Set report period to 1s
        bus.send(servo.set_report_period_frame(1000))

        # Measure time between two reports

        # Wait for first report
        first = bus.recv(timeout=2.0)
        assert first is not None, "No message received"

        # Wait for second report
        second = bus.recv(timeout=2.0)
        assert second is not None, "No message received"

        time_diff = second.timestamp - first.timestamp
        allowed_error = 0.02
        assert time_diff > 1 - allowed_error, (
            f"Time between reports was less than 1s: {time_diff} (allowed error: {allowed_error})"
        )

        # Reset filters
        bus.set_filters(None)

        # Restore report period
        bus.send(servo.set_report_period_frame(200))

        # Steer using pulse
        bus.send(servo.set_steering_pulse_frame(2000))
        time.sleep(2)
        bus.send(servo.set_steering_pulse_frame(1000))
        time.sleep(2)

        # Steer using angle
        bus.send(servo.set_steering_angle_frame(45))
        time.sleep(2)
        bus.send(servo.set_steering_angle_frame(-45))
        time.sleep(2)

        # Reverse direction then set same steering angle as before.
        # This should move servo 90 degrees total.
        bus.send(servo.set_reverse_direction())
        bus.send(servo.set_steering_angle_frame(0))
        time.sleep(2)
        bus.send(servo.set_steering_angle_frame(-45))
        time.sleep(2)

        # This should set the servo position to neutral by triggering the failsafe.
        bus.send(servo.set_failsafe(servo.FAILSAFE_ON, timeout_ms=100, pulse_mus=1500))

        # Restore settings
        bus.send(servo.set_reverse_direction())

        # Re-enable SBUS receiver city's communication
        bus.send(
            rover.set_comm_mode(
                city=City.SBUS_RECEIVER, mode=rover.CommMode.COMMUNICATE
            )
        )


if __name__ == "__main__":
    main()
