"""CAN interface configuration module."""

import argparse

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
        help="type of interface to use (default: socketcan)",
        default="socketcan",
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
    return can.ThreadSafeBus(
        interface=args.interface,
        channel=get_channel_name(args.interface, args.channel),
        bitrate=args.bitrate,
    )


def detect_available_interfaces() -> list[AutoDetectedConfig]:
    """Detect available CAN interfaces.

    Returns:
        list[AutoDetectedConfig]: List of available interface configurations.
    """
    return can.detect_available_configs(interfaces=["socketcan", "kvaser"])
