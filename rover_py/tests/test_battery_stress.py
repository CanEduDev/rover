import argparse
import csv
import datetime
import time
from pathlib import Path

import cantools
import numpy
from rover import Envelope, battery
from rover.can_interface import add_can_args, create_bus_from_args

# Test uses 3S battery, capacity 6000mAh and discharge rate 70C.
#
# Resistor configurations:
# 1: 0.500 ohm
# 2: 0.250 ohm
# 3: 0.167 ohm
# 4: 0.125 ohm
#
# Expected currents for 3S battery:
# 1: ~23A
# 2: ~45A
# 3: ~65A
# 4: ~83A


def main():
    parser = argparse.ArgumentParser(description="Battery stress test")
    add_can_args(parser)
    parser.add_argument("--dbc", type=str, default="rover.dbc", help="DBC file path")
    args = parser.parse_args()

    db = cantools.db.load_file(args.dbc)

    with create_bus_from_args(args) as bus:
        setup_test(bus)
        run_test(bus, db)
        teardown_test(bus)


def setup_test(bus):
    bus.set_filters(
        [
            {"can_id": Envelope.BATTERY_CELL_VOLTAGES, "can_mask": (1 << 11) - 1},
            {"can_id": Envelope.BATTERY_OUTPUT, "can_mask": (1 << 11) - 1},
        ]
    )

    bus.send(battery.set_reg_pwr_off_frame)
    bus.send(battery.set_pwr_off_frame)
    bus.send(battery.set_vbat_out_overcurrent_threshold_frame(95_000))

    # Set report period to 10 ms
    bus.send(battery.set_report_period_frame(10))

    # Wait for reg output to go to 0 volt
    time.sleep(2)


def run_test(bus, db):
    signal_value_map = {
        "CELL_1_VOLTAGE": [],
        "CELL_2_VOLTAGE": [],
        "CELL_3_VOLTAGE": [],
        "BATTERY_OUTPUT_VOLTAGE": [],
        "BATTERY_OUTPUT_CURRENT": [],
    }
    collect_data(bus, signal_value_map, db)
    print_data(signal_value_map)
    write_csv(signal_value_map)


def collect_data(bus, signal_value_map, db):
    # Start test
    bus.send(battery.set_pwr_on_frame)

    # Wait for voltage to stabilize
    time.sleep(0.5)

    # Clear receive buffer
    while bus.recv(timeout=0.005) is not None:
        pass

    # Run for 3 seconds
    start_time = time.time()
    while time.time() - start_time < 3:
        try:
            msg = bus.recv(timeout=1.0)

            if msg is None:
                continue

            decoded = db.decode_message(msg.arbitration_id, msg.data)
            if (
                msg.arbitration_id == Envelope.BATTERY_CELL_VOLTAGES
                and decoded["CELLS"] == "CELLS_1_TO_3"
            ):
                signal_value_map["CELL_1_VOLTAGE"].append(decoded["CELL_1_VOLTAGE"])
                signal_value_map["CELL_2_VOLTAGE"].append(decoded["CELL_2_VOLTAGE"])
                signal_value_map["CELL_3_VOLTAGE"].append(decoded["CELL_3_VOLTAGE"])
            elif msg.arbitration_id == Envelope.BATTERY_OUTPUT:
                signal_value_map["BATTERY_OUTPUT_VOLTAGE"].append(
                    decoded["BATTERY_OUTPUT_VOLTAGE"]
                )
                signal_value_map["BATTERY_OUTPUT_CURRENT"].append(
                    decoded["BATTERY_OUTPUT_CURRENT"]
                )

        except KeyboardInterrupt:
            break

    # Stop test
    bus.send(battery.set_pwr_off_frame)


def print_data(signal_value_map):
    # Print min, max, median, mean for all signals
    for signal in signal_value_map:
        values = signal_value_map[signal]
        if not values:
            continue
        print(
            f"{signal}: min: {min(values)}, max: {max(values)}, median: {numpy.median(values)}, mean: {numpy.mean(values)}"
        )


def write_csv(signal_value_map):
    # Write CSV file
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
    with Path(f"battery-stress-test_{formatted_datetime}.csv").open(
        "w", newline=""
    ) as csv_file:
        writer = csv.writer(csv_file)
        header = []
        values = []
        for signal in signal_value_map:
            header.append(signal)
            values.append(signal_value_map[signal])
        writer.writerow(header)
        writer.writerows(zip(*values, strict=False))


def teardown_test(bus):
    # Restore defaults
    bus.send(battery.set_report_period_frame(200))


if __name__ == "__main__":
    main()
