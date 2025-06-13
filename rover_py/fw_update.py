#!/usr/bin/env python3

import argparse
import sys
import time

import can
from flasher import flasher
from rover import rover

default_config = "system.json"
default_bindir = "binaries"


def main():
    args = parse_args()

    if args.bitrate != "125k":
        change_bitrate_125k(args)

    run_flasher(args)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Configure and flash Rover ECUs over CAN",
    )

    parser.add_argument(
        "--channel", default=0, type=int, help="CANlib CAN channel to use (default: 0)"
    )
    parser.add_argument(
        "--bitrate",
        default="125k",
        choices=["125k", "250k", "500k", "1m"],
        help="use this option if you have changed the default CAN bitrate",
    )

    subparsers = parser.add_subparsers(required=False, dest="subcommand")

    subparsers.add_parser("list", help="list online nodes")

    parser_system = subparsers.add_parser(
        "system", help="flash a complete Rover system"
    )
    parser_system.add_argument(
        "--config",
        default=default_config,
        help="flasher configuration file (default: system.json)",
    )
    parser_system.add_argument(
        "--binary-dir",
        default=default_bindir,
        help="binary file directory (default: binaries)",
    )
    parser_system.add_argument(
        "--skip-binaries",
        action="store_true",
        default=False,
        help="do not flash binaries, only update configuration",
    )
    parser_system.add_argument(
        "--skip-config",
        action="store_true",
        default=False,
        help="do not update configuration, flash binaries only",
    )

    parser_single = subparsers.add_parser(
        "single",
        description="Flash, configure or format single ECU.",
        help="(advanced) interact with a single ECU",
    )
    parser_single.add_argument("--id", type=int, help="node ID", required=True)
    parser_single.add_argument("--binary", help="path to node binary file")
    parser_single.add_argument("--config", help="path to node config file")
    parser_single.add_argument(
        "--format-fs", action="store_true", help="format node filesystem"
    )

    parser_recovery = subparsers.add_parser(
        "recover",
        description="(Advanced) Recover node stuck in bootloader",
        help="(advanced) recover node stuck in bootloader",
    )
    parser_recovery.add_argument(
        "--binary", help="path to node binary file", required=True
    )
    parser_recovery.add_argument(
        "--config", help="path to node config file", required=True
    )

    args = parser.parse_args()
    if not args.subcommand:
        parser.print_help()
        sys.exit(1)

    return args


def change_bitrate_125k(args):
    try:
        bus = can.ThreadSafeBus(
            interface=args.interface or "socketcan",
            channel=args.channel or "can0",
            bitrate=parse_bitrate_arg(args.bitrate),
        )

        bus.send(rover.change_bitrate_125kbit())
        bus.send(
            rover.restart_communication(
                skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
            )
        )
        bus.shutdown()

    except Exception as e:
        print(
            f"error: couldn't change bitrate from {args.bitrate} to 125k: {e}",
            file=sys.stderr,
        )
        sys.exit(1)

    time.sleep(0.1)


def run_flasher(args):
    with can.ThreadSafeBus(
        interface=args.interface or "socketcan",
        channel=args.channel or "can0",
        bitrate=parse_bitrate_arg(args.bitrate),
    ) as bus:
        try:
            print("Running flasher...")

            if args.subcommand == "list":
                f = flasher.Flasher(bus)
                node_ids = f.detect_online_nodes()
                print("Found nodes:")
                for id in node_ids:
                    print(f"  {id}: {rover.City(id).name}")

                sys.exit(0)

            elif args.subcommand == "system":
                run_system(bus, args)

            elif args.subcommand == "single":
                run_single(bus, args)

            elif args.subcommand == "recover":
                print("Try to enter recovery mode...")
                flasher.Flasher(bus).enter_recovery_mode(args.binary, args.config)
                sys.exit(0)

        except Exception as e:
            # restart all nodes
            bus.send(rover.set_action_mode(mode=rover.ActionMode.RESET))
            print(f"error: flashing failed: {e}", file=sys.stderr)
            sys.exit(1)


def run_single(bus, args):
    if args.id == 0:
        print("id 0 is reserved by the flasher.", file=sys.stderr)
        sys.exit(1)

    if not (args.binary or args.config or args.format_fs):
        print(
            "Incorrect arguments. At least one of --binary, --config or --format-fs must be given.",
            file=sys.stderr,
        )
        sys.exit(1)

    f = flasher.Flasher(bus)
    if args.format_fs:
        f.format_fs(args.id)
    else:
        f.run_single(args.id, binary_file=args.binary, config_file=args.config)


def run_system(bus, args):
    print("Verifying system configuration...")
    bindir_arg = default_bindir
    config_arg = default_config
    if args.subcommand == "system":
        bindir_arg = args.binary_dir
        config_arg = args.config

    try:
        config = flasher.FlasherConfig(
            config_arg,
            bindir_arg,
            skip_binaries=args.skip_binaries,
            skip_config=args.skip_config,
        )
    except Exception as e:
        print(f"error verifying {args.config}: {e}", file=sys.stderr)
        sys.exit(1)

    flasher.Flasher(bus, config).run()


def parse_bitrate_arg(bitrate):
    if bitrate == "125k":
        return 125000
    if bitrate == "250k":
        return 250000
    if bitrate == "500k":
        return 500000
    if bitrate == "1m":
        return 1000000


if __name__ == "__main__":
    main()
