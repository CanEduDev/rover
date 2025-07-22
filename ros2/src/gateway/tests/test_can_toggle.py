import time

from rover import rover
from rover.can_interface import create_bus


def test_action_mode_freeze():
    with create_bus(channel="can1", interface="socketcan", bitrate=125_000) as bus:
        # set_action_mode to FREEZE
        msg = rover.set_action_mode(mode=rover.ActionMode.FREEZE)
        bus.send(msg)
        time.sleep(1)

        # reset
        msg = rover.set_action_mode(mode=rover.ActionMode.RUN)
        bus.send(msg)
        time.sleep(1)


def test_comm_mode_listen_only():
    with create_bus(channel="can1", interface="socketcan", bitrate=125_000) as bus:
        # set_comm_mode to LISTEN_ONLY
        msg = rover.set_comm_mode(mode=rover.CommMode.LISTEN_ONLY)
        bus.send(msg)
        time.sleep(1)

        reset_comm_mode(bus)


def test_comm_mode_silent():
    with create_bus(channel="can1", interface="socketcan", bitrate=125_000) as bus:
        # set_comm_mode to SILENT
        msg = rover.set_comm_mode(mode=rover.CommMode.SILENT)
        bus.send(msg)
        time.sleep(1)

        reset_comm_mode(bus)


def reset_comm_mode(bus):
    # set_comm_mode to COMMUNICATE
    msg = rover.set_comm_mode(mode=rover.CommMode.COMMUNICATE)
    bus.send(msg)
    time.sleep(1)


def main():
    print("Testing action mode freeze")
    test_action_mode_freeze()
    print("Testing comm mode listen only")
    test_comm_mode_listen_only()
    print("Testing comm mode silent")
    test_comm_mode_silent()
    print("Done")


if __name__ == "__main__":
    main()
