import sys
from time import sleep

import can
from rover import rover

try:
    # Start with 125 kbit/s
    bus = can.ThreadSafeBus(
        interface="socketcan",  # or "kvaser" for Kvaser interface
        channel="can0",  # or 0 for first Kvaser channel
        bitrate=125000,
    )

    # Send multiple default letters to make sure every node receives one at startup
    for _ in range(5):
        bus.send(rover.default_letter())
        sleep(0.05)

    bus.send(rover.give_base_number())
    msg = bus.recv(timeout=1.0)
    sleep(0.5)  # Wait for responses

    # Switch to 500 kbit/s
    print("Switching to 500 kbit/s")
    bus.send(rover.change_bitrate_500kbit())
    bus.send(
        rover.restart_communication(
            skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
        )
    )

    bus.shutdown()
    bus = can.ThreadSafeBus(
        interface="socketcan",  # or "kvaser" for Kvaser interface
        channel="can0",  # or 0 for first Kvaser channel
        bitrate=500000,
    )
    sleep(0.1)  # give time for changing bitrate

    bus.send(rover.default_letter())
    bus.send(rover.give_base_number())
    msg = bus.recv(timeout=1.0)
    sleep(0.5)  # Wait for responses

    print("Switching to 125 kbit/s")
    bus.send(rover.change_bitrate_125kbit())
    bus.send(
        rover.restart_communication(
            skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
        )
    )

    bus.shutdown()
    bus = can.ThreadSafeBus(
        interface="socketcan",  # or "kvaser" for Kvaser interface
        channel="can0",  # or 0 for first Kvaser channel
        bitrate=125000,
    )

    bus.send(rover.default_letter())
    bus.send(rover.give_base_number())
    msg = bus.recv(timeout=1.0)
    sleep(0.5)  # Wait for responses

    print("Done")
    bus.shutdown()

except Exception as e:
    print(
        "Test timed out. make sure node is on the CAN bus and has a starting bitrate of 125kbit/s",
        file=sys.stderr,
    )
    sys.exit(1)
