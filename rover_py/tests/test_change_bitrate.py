import argparse
import sys
from time import sleep

import can
from rover import rover
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser(description="Test changing CAN bitrate")
    add_can_args(parser)
    args = parser.parse_args()

    try:
        # Start with 125 kbit/s
        args.bitrate = 125000
        with create_bus_from_args(args) as bus:
            # Switch to 500 kbit/s
            print("Switching to 500 kbit/s")
            bus.send(rover.change_bitrate_500kbit())
            bus.send(
                rover.restart_communication(
                    skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
                )
            )

        sleep(0.1)  # give time for changing bitrate

        # Create new bus with 500 kbit/s
        args.bitrate = 500000
        with create_bus_from_args(args) as bus:
            try:
                # Send and receive messages, listen for error frames
                for _ in range(20):
                    bus.send(rover.default_letter())
                    sleep(0.05)
                    msg = bus.recv(timeout=0.1)
                    if msg.is_error_frame:
                        print("Error: Received error frame on CAN bus", file=sys.stderr)
                        sys.exit(1)
            except can.CanOperationError:
                print("Error: Failed to send default letter", file=sys.stderr)
                sys.exit(1)

            # Switch back to 125 kbit/s
            print("Switching to 125 kbit/s")
            bus.send(rover.change_bitrate_125kbit())
            bus.send(
                rover.restart_communication(
                    skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
                )
            )

        sleep(0.1)  # give time for changing bitrate

        # Create new bus with 125 kbit/s
        args.bitrate = 125000
        with create_bus_from_args(args) as bus:
            try:
                # Send and receive messages, listen for error frames
                for _ in range(20):
                    bus.send(rover.default_letter())
                    msg = bus.recv(timeout=1.0)
                    if msg.is_error_frame:
                        print("Error: Received error frame on CAN bus", file=sys.stderr)
                        sys.exit(1)
            except can.CanOperationError:
                print("Error: Failed to send default letter", file=sys.stderr)
                sys.exit(1)

        print("Done")

    except Exception as e:
        print(
            f"Test failed: {str(e)}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
