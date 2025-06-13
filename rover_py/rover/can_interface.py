"""CAN interface configuration module."""

import argparse
import subprocess

import can
from can.typechecking import AutoDetectedConfig


def add_can_args(parser: argparse.ArgumentParser) -> None:
    """Add CAN interface arguments to an argument parser.

    Args:
        parser: The argument parser to add arguments to.
    """
    parser.add_argument(
        "-i",
        "--interface",
        help="type of interface to use (default: kvaser)",
        default="kvaser",
        choices=["kvaser", "socketcan"],
    )
    parser.add_argument(
        "-c",
        "--channel",
        help="channel to use (default: can0 for socketcan, 0 for others)",
    )
    parser.add_argument(
        "-b",
        "--bitrate",
        type=int,
        default=125000,
        help="CAN bus bitrate (default: 125000)",
    )


def get_channel_name(interface: str, channel: str | None = None) -> str:
    """Get the channel name based on interface type.

    Args:
        interface: The CAN interface type (e.g., 'socketcan', 'kvaser').
        channel: Optional channel override.

    Returns:
        str: The channel name to use.
    """
    if channel is not None:
        return channel

    return "can0" if interface == "socketcan" else "0"


def create_bus_from_args(args: argparse.Namespace) -> can.ThreadSafeBus:
    """Create a CAN bus from command line arguments.

    Args:
        args: Parsed command line arguments containing interface, channel, and bitrate.

    Returns:
        can.ThreadSafeBus: The configured CAN bus.
    """
    if args.interface == "socketcan":
        setup_socketcan_interface(args.interface, args.channel, args.bitrate)

    return can.ThreadSafeBus(
        interface=args.interface,
        channel=get_channel_name(args.interface, args.channel),
        bitrate=args.bitrate,
    )


def create_bus(
    interface: str, channel: str | None = None, bitrate: int = 125000
) -> can.ThreadSafeBus:
    """Create a CAN bus.

    Args:
        interface: The CAN interface type (e.g., 'socketcan', 'kvaser').
        channel: The channel to use.
        bitrate: The CAN bus bitrate.

    Returns:
        can.ThreadSafeBus: The configured CAN bus.
    """

    if interface == "socketcan":
        setup_socketcan_interface(interface, channel, bitrate)

    return can.ThreadSafeBus(
        interface=interface,
        channel=get_channel_name(interface, channel),
        bitrate=bitrate,
    )


def detect_available_interfaces() -> list[AutoDetectedConfig]:
    """Detect available CAN interfaces.

    Returns:
        list[AutoDetectedConfig]: List of available interface configurations.
    """
    return can.detect_available_configs(interfaces=["socketcan", "kvaser"])


def setup_socketcan_interface(interface, channel, bitrate):
    """Set up the socketcan interface using subprocess and ip commands."""
    print(
        f"Elevated permissions required to setup socketcan interface {get_channel_name(interface, channel)}"
    )
    subprocess.run(
        [
            "sudo",
            "ip",
            "link",
            "set",
            get_channel_name(interface, channel),
            "down",
        ],
        check=True,
    )
    subprocess.run(
        [
            "sudo",
            "ip",
            "link",
            "set",
            get_channel_name(interface, channel),
            "type",
            "can",
            "bitrate",
            str(bitrate),
            "restart-ms",
            "100",
        ],
        check=True,
    )
    subprocess.run(
        [
            "sudo",
            "ip",
            "link",
            "set",
            get_channel_name(interface, channel),
            "up",
        ],
        check=True,
    )
