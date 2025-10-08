#!/usr/bin/env python3
"""
Test buzzer functionality by playing "Frère Jacques" melody.

This test sends CAN messages to the buzzer system to play the classic
French nursery rhyme "Frère Jacques" with proper timing to avoid
queue overflow.
"""

import argparse
import time

from rover import buzzer
from rover.can_interface import add_can_args, create_bus_from_args


def main():
    parser = argparse.ArgumentParser(
        description="Test buzzer with Frère Jacques melody"
    )
    add_can_args(parser)
    args = parser.parse_args()

    D4 = 294  # noqa: N806
    G4 = 392  # noqa: N806
    A4 = 440  # noqa: N806
    B4 = 494  # noqa: N806
    C5 = 523  # noqa: N806
    D5 = 587  # noqa: N806
    E5 = 660  # noqa: N806

    mute_duration = 10
    eighth_note = 250
    quarter_note = 500
    half_note = 1000

    volume = 30

    melody = [
        # "Frère Jacques, Frère Jacques"
        (G4, quarter_note, volume),
        (A4, quarter_note, volume),
        (B4, quarter_note, volume),
        (G4, quarter_note - mute_duration, volume),
        (0, mute_duration, 0),
        (G4, quarter_note, volume),
        (A4, quarter_note, volume),
        (B4, quarter_note, volume),
        (G4, quarter_note - mute_duration, volume),
        (0, mute_duration, 0),
        # "Dormez-vous? Dormez-vous?"
        (B4, quarter_note, volume),
        (C5, quarter_note, volume),
        (D5, half_note - mute_duration, volume),
        (0, mute_duration, 0),
        (B4, quarter_note, volume),
        (C5, quarter_note, volume),
        (D5, half_note - mute_duration, volume),
        (0, mute_duration, 0),
        # "Sonnez les matines, Sonnez les matines"
        (D5, eighth_note, volume),
        (E5, eighth_note, volume),
        (D5, eighth_note, volume),
        (C5, eighth_note, volume),
        (B4, quarter_note, volume),
        (G4, quarter_note - mute_duration, volume),
        (0, mute_duration, 0),
        (D5, eighth_note, volume),
        (E5, eighth_note, volume),
        (D5, eighth_note, volume),
        (C5, eighth_note, volume),
        (B4, quarter_note, volume),
        (G4, quarter_note - mute_duration, volume),
        (0, mute_duration, 0),
        # "Ding, dang, dong, Ding, dang, dong"
        (G4, quarter_note, volume),
        (D4, quarter_note, volume),
        (G4, half_note - mute_duration, volume),
        (0, mute_duration, 0),
        (G4, quarter_note, volume),
        (D4, quarter_note, volume),
        (G4, half_note - mute_duration, volume),
        (0, mute_duration, 0),
    ]

    with create_bus_from_args(args) as bus:
        print("Playing Frère Jacques melody on buzzer...")
        print("Note: Make sure the sbus-receiver is running and buzzer is connected!")

        # Test queue functionality by sending notes and adding delay every 8 notes

        time_to_wait = 0
        for i, (frequency, duration, volume) in enumerate(melody):
            print(
                f"Playing note {i + 1}/{len(melody)}: {frequency}Hz for {duration}ms at volume {volume}"
            )

            # Send buzzer sound message
            bus.send(buzzer.set_buzzer_sound_frame(frequency, duration, volume))
            time_to_wait += duration

            # Add delay every 8 notes to let the queue process
            if i % 7 == 0:
                time.sleep(time_to_wait / 1000.0)
                time_to_wait = 0

        time.sleep(time_to_wait / 1000.0)  # Wait for the last note to finish
        print("Melody complete! All notes have been played.")
        print("Frère Jacques test completed successfully!")


if __name__ == "__main__":
    main()
