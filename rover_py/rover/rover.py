import can

from . import BASE_NUMBER, ActionMode, City, CommMode


def default_letter():
    return can.Message(
        arbitration_id=2031,
        data=[0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA],
        is_extended_id=False,
    )


def set_comm_mode(city=City.ALL_CITIES, mode=CommMode.KEEP_CURRENT):
    return can.Message(
        arbitration_id=0,
        dlc=8,
        data=[city, 0, 0, mode, 0, 0, 0, 0],
        is_extended_id=False,
    )


def set_action_mode(city=City.ALL_CITIES, mode=ActionMode.KEEP_CURRENT):
    return can.Message(
        arbitration_id=0,
        dlc=8,
        data=[city, 0, mode, 0, 0, 0, 0, 0],
        is_extended_id=False,
    )


# Give base number and ask for response page
# Response pages are 0 for EAN and 1 for serial number.
# Besides these two, applications can define their own response pages.
def give_base_number(city=City.ALL_CITIES, base_no=BASE_NUMBER, response_page=0):
    base_no_data = list(base_no.to_bytes(4, "little"))
    data = [city, 1, response_page, 0] + base_no_data
    return can.Message(arbitration_id=0, dlc=8, data=data, is_extended_id=False)


def assign_envelope(city, envelope, folder):
    envelope_data = list(envelope.to_bytes(4, "little"))
    data = [city, 2] + envelope_data + [folder, 0x3]
    return can.Message(arbitration_id=0, dlc=8, data=data, is_extended_id=False)


def change_bit_timing(prescaler, tq, phase_seg2, sjw, city=City.ALL_CITIES):
    return can.Message(
        arbitration_id=0,
        dlc=8,
        data=[city, 8, 0, 0, prescaler, tq, phase_seg2, sjw],
        is_extended_id=False,
    )


def change_bitrate_125kbit(city=City.ALL_CITIES):
    return change_bit_timing(prescaler=18, tq=16, phase_seg2=2, sjw=1, city=city)


def change_bitrate_500kbit(city=City.ALL_CITIES):
    return change_bit_timing(prescaler=9, tq=8, phase_seg2=1, sjw=1, city=city)


def change_bitrate_1mbit(city=City.ALL_CITIES):
    return change_bit_timing(prescaler=4, tq=9, phase_seg2=1, sjw=1, city=city)


# Reset communication mode. Useful when switching bitrates.
# Optionally skip startup sequence after reset.
def restart_communication(
    city=City.ALL_CITIES, skip_startup=False, comm_mode=CommMode.SILENT
):
    frame = set_comm_mode(city=city, mode=comm_mode)
    # See CanKingdom v4 KP0 for definition
    reset_flag = 1 << 2
    if skip_startup:
        skip_listen_flag = 1 << 3
        skip_wait_flag = 1 << 5
    else:
        skip_listen_flag = 1 << 4
        skip_wait_flag = 1 << 6

    frame.data[3] |= reset_flag
    frame.data[3] |= skip_listen_flag
    frame.data[3] |= skip_wait_flag

    return frame
