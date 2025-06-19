#!/usr/bin/env python3

import argparse
import sys
import traceback

from flasher import flasher
from rover import rover
from rover.can_interface import add_can_args

default_config = "system.json"
default_bindir = "binaries"


def main():
    args = parse_args()
    run_flasher(args)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Configure and flash Rover ECUs over CAN",
    )
    add_can_args(parser)

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


def run_flasher(args):
    try:
        f = flasher.Flasher(args.interface, args.channel, args.bitrate)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        print(
            "Please specify interface and channel explicitly using -i and -c. See --help for more information.",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        print("Running flasher...")

        if args.subcommand == "list":
            node_ids = f.detect_online_nodes()
            print("Found nodes:")
            for id in node_ids:
                print(f"  {id}: {rover.City(id).name}")

            sys.exit(0)

        elif args.subcommand == "system":
            run_system(f, args)

        elif args.subcommand == "single":
            run_single(f, args)

        elif args.subcommand == "recover":
            print("Try to enter recovery mode...")
            f.enter_recovery_mode(args.binary, args.config)
            sys.exit(0)

    except Exception as e:
        traceback.print_exc()
        print(f"error: flashing failed: {e}", file=sys.stderr)
        sys.exit(1)


def run_single(f: flasher.Flasher, args):
    if args.id == 0:
        print("id 0 is reserved by the flasher.", file=sys.stderr)
        sys.exit(1)

    if not (args.binary or args.config or args.format_fs):
        print(
            "Incorrect arguments. At least one of --binary, --config or --format-fs must be given.",
            file=sys.stderr,
        )
        sys.exit(1)

    if args.format_fs:
        f.format_fs(args.id)
    else:
        f.run_single(args.id, binary_file=args.binary, config_file=args.config)


def run_system(f: flasher.Flasher, args):
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

    f.config = config
    f.run()


if __name__ == "__main__":
    main()
