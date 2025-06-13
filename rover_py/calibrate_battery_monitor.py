import argparse
import sys

import can
from rover import Envelope, battery
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser(description="Calibrate battery monitor")
    add_can_args(parser)
    args = parser.parse_args()

    calibration_voltage_mv = 24_000
    input(
        f"""Connect power board to DC power supply with {calibration_voltage_mv / 1000}V on all cells.
Connect CAN interface to power board. Make sure the connection is terminated with a 120 ohm resistance on at least one end.
Turn power on and press Enter to continue.
    """
    )

    try:
        bus = create_bus_from_args(args)
        bus.set_filters(
            [{"can_id": Envelope.BATTERY_CELL_VOLTAGES, "can_mask": (1 << 11) - 1}]
        )
        # Clear receive buffer
        while bus.recv(timeout=0.01) is not None:
            pass

    except Exception as e:
        print(f"error: failed to create CAN bus: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        start_voltage_mv = read_cell0_voltage(bus)
        accepted_error = 1000
        error = abs(calibration_voltage_mv - start_voltage_mv)

        if error > accepted_error:
            ans = input(
                f"""warning: measured startup voltage is {start_voltage_mv}mV.
Make sure you set the voltage to {calibration_voltage_mv / 1000}V.
Continue? [y/N] >
"""
            )
            if ans.lower() != "y":
                sys.exit(0)

        bus.send(battery.calibration_frame(voltage=calibration_voltage_mv))

        voltage_mv = read_cell0_voltage(bus)

        accepted_error = 100
        error = abs(calibration_voltage_mv - voltage_mv)
        if error > accepted_error:
            print(
                f"error: calibration failed: calibration error {error}mV is too large."
            )
            sys.exit(1)

        print("Calibration succeeded.")

    except can.CanError as e:
        print(f"error: CAN error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        bus.shutdown()


def read_cell0_voltage(bus):
    # Cell message is sent in two frames.
    # We want the first frame, where byte 0 == 0.
    msg = bus.recv(timeout=1.0)
    if msg.arbitration_id != Envelope.BATTERY_CELL_VOLTAGES:
        raise can.CanError("Unexpected message ID")

    if msg.data[0] == 1:
        msg = bus.recv(timeout=1.0)
        if msg.arbitration_id != Envelope.BATTERY_CELL_VOLTAGES:
            raise can.CanError("Unexpected message ID")

    return int.from_bytes(msg.data[1:3], byteorder="little")


if __name__ == "__main__":
    main()
