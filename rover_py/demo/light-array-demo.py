import argparse

import can
import cantools
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser(
        description="Send light states based on servo position from a DBC file."
    )
    add_can_args(parser)
    parser.add_argument("dbc_file", type=str, help="Path to the DBC file")

    args = parser.parse_args()

    db = cantools.db.load_file(args.dbc_file)

    servo_msg = db.get_message_by_name("SERVO_POSITION")  # type: ignore
    light_msg = db.get_message_by_name("LIGHT_ARRAY_FRONT_STATE")  # type: ignore

    bus = create_bus_from_args(args)

    last_light_state = angle_to_lights(0)

    try:
        for msg in bus:
            if msg is None or msg.arbitration_id != servo_msg.frame_id:
                continue

            decoded = servo_msg.decode(msg.data)
            angle = decoded["SERVO_POSITION"]  # type: ignore

            light_signals = angle_to_lights(angle)

            if light_signals == last_light_state:
                continue

            encoded_light_data = light_msg.encode(light_signals)

            can_msg = can.Message(
                arbitration_id=light_msg.frame_id,
                data=encoded_light_data,
                is_extended_id=light_msg.is_extended_frame,
            )

            bus.send(can_msg)

            last_light_state = light_signals

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"error: {e}")

    finally:
        bus.shutdown()


def angle_to_lights(angle):
    first_threshold_deg = 10
    second_threshold_deg = 25

    """Converts angle [-45, 45] to light state dictionary."""
    lights = {
        "LIGHT_LEFT": 0,
        "LIGHT_MID_LEFT": 0,
        "LIGHT_MID_RIGHT": 0,
        "LIGHT_RIGHT": 0,
    }

    if angle <= -first_threshold_deg:
        lights["LIGHT_MID_LEFT"] = 1
        if angle <= -second_threshold_deg:
            lights["LIGHT_LEFT"] = 1
    elif angle >= first_threshold_deg:
        lights["LIGHT_MID_RIGHT"] = 1
        if angle >= second_threshold_deg:
            lights["LIGHT_RIGHT"] = 1

    return lights


if __name__ == "__main__":
    main()
