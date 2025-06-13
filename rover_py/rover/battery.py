# Battery board testing frames

from enum import IntEnum

import can

from . import Envelope


class JumperConfig(IntEnum):
    X11_OFF_X12_OFF = 0x00
    X11_ON_X12_OFF = 0x01
    X11_OFF_X12_ON = 0x02
    X11_ON_X12_ON = 0x03


def set_jumper_conf_frame(jumper: JumperConfig):
    data = [jumper]
    return can.Message(
        arbitration_id=Envelope.BATTERY_JUMPER_CONFIG,
        dlc=1,
        data=data,
        is_extended_id=False,
    )


def set_vbat_out_overcurrent_threshold_frame(current_ma):
    data = list(current_ma.to_bytes(4, "little"))
    return can.Message(
        arbitration_id=Envelope.BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD,
        dlc=4,
        data=data,
        is_extended_id=False,
    )


def set_reg_out_overcurrent_threshold_frame(current_ma):
    data = list(current_ma.to_bytes(4, "little"))
    return can.Message(
        arbitration_id=Envelope.BATTERY_REG_OUT_OVERCURRENT_THRESHOLD,
        dlc=4,
        data=data,
        is_extended_id=False,
    )


def set_reg_out_voltage_frame(voltage_mv):
    data = list(voltage_mv.to_bytes(4, "little"))
    return can.Message(
        arbitration_id=Envelope.BATTERY_REG_OUT_VOLTAGE,
        dlc=4,
        data=data,
        is_extended_id=False,
    )


set_reg_pwr_off_frame = can.Message(
    arbitration_id=Envelope.BATTERY_OUTPUT_ON_OFF,
    dlc=2,
    data=[0xFF, 0],
    is_extended_id=False,
)  # Set regulated power out OFF

set_reg_pwr_on_frame = can.Message(
    arbitration_id=Envelope.BATTERY_OUTPUT_ON_OFF,
    dlc=2,
    data=[0xFF, 1],
    is_extended_id=False,
)  # Set regulated power out ON

set_pwr_off_frame = can.Message(
    arbitration_id=Envelope.BATTERY_OUTPUT_ON_OFF,
    dlc=2,
    data=[0, 0xFF],
    is_extended_id=False,
)  # Set power out OFF

set_pwr_on_frame = can.Message(
    arbitration_id=Envelope.BATTERY_OUTPUT_ON_OFF,
    dlc=2,
    data=[1, 0xFF],
    is_extended_id=False,
)  # Set power out ON


def set_low_voltage_cutoff_frame(cutoff_mv):
    data = list(cutoff_mv.to_bytes(2, "little"))
    return can.Message(
        arbitration_id=Envelope.BATTERY_LOW_VOLTAGE_CUTOFF,
        dlc=2,
        data=data,
        is_extended_id=False,
    )


def set_report_period_frame(time_ms):
    data = list(time_ms.to_bytes(2, "little"))
    return can.Message(
        arbitration_id=Envelope.BATTERY_REPORT_FREQUENCY,
        dlc=2,
        data=data,
        is_extended_id=False,
    )


def calibration_frame(voltage):
    data = list(voltage.to_bytes(2, "little"))
    return can.Message(
        arbitration_id=Envelope.BATTERY_CELL_CALIBRATION,
        dlc=2,
        data=data,
        is_extended_id=False,
    )
