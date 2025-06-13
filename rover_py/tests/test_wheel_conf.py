import argparse

from rover import City, Envelope, wheel
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser(description="Test wheel configuration")
    add_can_args(parser)
    args = parser.parse_args()

    with create_bus_from_args(args) as bus:
        bus.set_filters(
            [{"can_id": Envelope.WHEEL_FRONT_LEFT_SPEED, "can_mask": (1 << 11) - 1}]
        )

        print("Testing report period...")
        # Set report period to 1s
        bus.send(wheel.set_report_period_frame(1000, wheel_id=City.WHEEL_FRONT_LEFT))

        # Clear receive buffer
        while bus.recv(timeout=0.1) is not None:
            pass

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
        bus.send(wheel.set_report_period_frame(200, wheel_id=City.WHEEL_FRONT_LEFT))

        print(
            "Testing setting incorrect wheel parameters. Data should be abnormally high. Check using logger."
        )
        bus.send(wheel.set_wheel_parameters_frame(5, 1, wheel_id=City.WHEEL_FRONT_LEFT))

        input("Press Enter to continue...")

        # Restore defaults
        bus.send(
            wheel.set_wheel_parameters_frame(45, 0.16, wheel_id=City.WHEEL_FRONT_LEFT)
        )


if __name__ == "__main__":
    main()
