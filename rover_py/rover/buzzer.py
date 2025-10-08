# Buzzer testing frames
import can

from . import Envelope


def set_buzzer_sound_frame(frequency_hz, duration_ms, volume):
    """Create a CAN message to play a buzzer sound.

    Args:
        frequency_hz: Sound frequency in Hz
        duration_ms: Sound duration in milliseconds
        volume: Sound volume (0-100)

    Returns:
        can.Message: CAN message for buzzer sound
    """
    data = (
        list(frequency_hz.to_bytes(2, "little"))
        + list(duration_ms.to_bytes(2, "little"))
        + list(volume.to_bytes(2, "little"))
    )

    return can.Message(
        arbitration_id=Envelope.BUZZER_SOUND,
        dlc=6,
        data=data,
        is_extended_id=False,
    )
