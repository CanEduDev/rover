import time

import can
from rover import Envelope, rover, wheel

bus = can.ThreadSafeBus(
    interface="socketcan",  # or "kvaser" for Kvaser interface
    channel="can0",  # or 0 for first Kvaser channel
    bitrate=125000,
)

rover.start(bus)

print("Testing report period...")
# Set report period to 1s
bus.send(wheel.set_report_period_frame(1000))

# Measure time between two reports
t_before = time.time()

# Wait for two consecutive speed reports
msg = bus.recv(timeout=2.0)
while msg and msg.arbitration_id != Envelope.WHEEL_FRONT_LEFT_SPEED:
    msg = bus.recv(timeout=2.0)

msg = bus.recv(timeout=2.0)
while msg and msg.arbitration_id != Envelope.WHEEL_FRONT_LEFT_SPEED:
    msg = bus.recv(timeout=2.0)

t_after = time.time()

assert t_after - t_before > 1

# Restore report period
bus.send(wheel.set_report_period_frame(200))

print(
    "Testing setting incorrect wheel parameters. Data should be abnormally high. Check using logger."
)
bus.send(wheel.set_wheel_parameters_frame(5, 1))

input("Press Enter to continue...")

# Restore defaults
bus.send(wheel.set_wheel_parameters_frame(45, 0.16))

bus.shutdown()
