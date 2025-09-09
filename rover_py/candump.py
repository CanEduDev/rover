import argparse
import sys
import time
from pathlib import Path

import can
import cantools
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser(
        description="dump rover CAN messages in DBC format"
    )
    parser.add_argument(
        "-l",
        "--list",
        action="store_true",
        help="list connected CAN interfaces and exit",
    )
    add_can_args(parser)
    parser.add_argument(
        "--dbc",
        metavar="DBC_FILE",
        default=None,
        help="CAN DBC file for decoding messages",
    )
    parser.add_argument(
        "--log", metavar="FILE", default=None, help="log output to file"
    )

    args = parser.parse_args()
    if args.list:
        configs = can.detect_available_configs(interfaces=["kvaser", "socketcan"])
        print("Found interfaces:")
        for interface in configs:
            print(f"\t{interface}")
        sys.exit(0)

    try:
        bus = create_bus_from_args(args)
    except Exception:
        print(
            "note: use --list to see available interfaces, then specify interface with -i",
            file=sys.stderr,
        )
        sys.exit(1)

    Dumper(bus, args.dbc, args.log).start()


class Dumper:
    def __init__(self, bus, dbc, log):
        self.bus = bus
        self.db = None
        self.logger = None

        if dbc is not None:
            try:
                self.db = cantools.db.load_file(dbc)
            except Exception as e:
                print(f"error: failed to load DBC file: {e}", file=sys.stderr)
                self.bus.shutdown()
                sys.exit(1)

        if log is not None:
            try:
                log_path = Path(log)
                log_path.parent.mkdir(parents=True, exist_ok=True)
                self.logger = can.Logger(log)
            except Exception as e:
                print(f"error: failed to create log file: {e}", file=sys.stderr)
                self.bus.shutdown()
                sys.exit(1)

    def start(self):
        notifier = can.Notifier(self.bus, [CanPrinter(self.db)])
        if self.logger:
            notifier.add_listener(self.logger)

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            notifier.stop()
            self.bus.shutdown()


class CanPrinter(can.Listener):
    def __init__(self, db):
        self.db = db

    def on_message_received(self, msg):
        print(msg)
        if self.db:
            try:
                print("\t", self.db.decode_message(msg.arbitration_id, msg.data))
            except KeyError:  # not decodable
                pass


if __name__ == "__main__":
    main()
