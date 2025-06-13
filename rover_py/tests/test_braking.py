import argparse
from time import sleep

from rover import City, rover, servo
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser()
    add_can_args(parser)
    args = parser.parse_args()

    with create_bus_from_args(args) as bus:
        # Disable SBUS receiver city's communication
        bus.send(
            rover.set_comm_mode(city=City.SBUS_RECEIVER, mode=rover.CommMode.SILENT)
        )
        sleep(1)

        # Disable failsafe
        bus.send(servo.set_failsafe(servo.FAILSAFE_OFF, city=City.MOTOR))

        # Accelerate
        bus.send(servo.set_throttle_pulse_frame(1600))
        sleep(1)

        # Brake
        bus.send(servo.set_throttle_pulse_frame(1000))
        sleep(1)

        # Go back to neutral in order to start reversing
        bus.send(servo.set_throttle_pulse_frame(1500))
        sleep(1)

        # Reverse
        bus.send(servo.set_throttle_pulse_frame(1400))
        sleep(1)

        # Back to neutral. There is no braking in reverse mode.
        bus.send(servo.set_throttle_pulse_frame(1500))

        # Re-enable failsafe
        bus.send(servo.set_failsafe(servo.FAILSAFE_ON, city=City.MOTOR))

        # Re-enable SBUS receiver city's communication
        bus.send(
            rover.set_comm_mode(
                city=City.SBUS_RECEIVER, mode=rover.CommMode.COMMUNICATE
            )
        )
        sleep(1)


if __name__ == "__main__":
    main()
