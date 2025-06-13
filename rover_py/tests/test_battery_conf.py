import argparse
import time

from rover import Envelope, battery
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser()
    add_can_args(parser)
    args = parser.parse_args()

    with create_bus_from_args(args) as bus:
        # Setup defaults
        bus.send(battery.set_reg_out_voltage_frame(5000))
        bus.send(battery.set_jumper_conf_frame(battery.JumperConfig.X11_ON_X12_ON))
        time.sleep(2)

        bus.set_filters(
            [{"can_id": Envelope.BATTERY_OUTPUT, "can_mask": (1 << 11) - 1}]
        )
        # Set report period to 1s
        bus.send(battery.set_report_period_frame(1000))

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
        bus.send(battery.set_report_period_frame(200))

        # Toggle regulated output power on / off
        bus.send(battery.set_reg_pwr_off_frame)
        time.sleep(2)
        bus.send(battery.set_reg_pwr_on_frame)
        time.sleep(2)

        # Toggle main power on / off
        bus.send(battery.set_pwr_off_frame)
        time.sleep(2)
        bus.send(battery.set_pwr_on_frame)
        time.sleep(2)

        # This should cause reg output to turn off due to over current protection
        # NOTE: load must be connected to reg out to see this.
        bus.send(battery.set_reg_out_overcurrent_threshold_frame(0))
        time.sleep(2)
        bus.send(battery.set_reg_out_overcurrent_threshold_frame(8000))
        bus.send(battery.set_reg_pwr_on_frame)
        time.sleep(2)

        # This should cause vbat out to turn off due to over current protection
        # NOTE: load must be connected to vbat out to see this.
        bus.send(battery.set_vbat_out_overcurrent_threshold_frame(0))
        time.sleep(2)
        bus.send(battery.set_vbat_out_overcurrent_threshold_frame(49500))
        bus.send(battery.set_pwr_on_frame)
        time.sleep(2)

        # This should cause power to turn off due to low-voltage cutoff
        # NOTE: cells must be connected for this to work.
        bus.send(battery.set_low_voltage_cutoff_frame(4200))
        time.sleep(2)

        # Restore defaults
        bus.send(battery.set_low_voltage_cutoff_frame(3000))
        bus.send(battery.set_pwr_on_frame)


if __name__ == "__main__":
    main()
