import time

import can
import keyboard
from rover import Envelope, rover, servo

bus = can.ThreadSafeBus(
    interface="socketcan",  # or "kvaser" for Kvaser interface
    channel="can0",  # or 0 for first Kvaser channel
    bitrate=125000,
)

bus.set_filters([{"can_id": Envelope.STEERING, "can_mask": Envelope.STEERING}])

rover.start(bus)

throttle = 1500
steering = 1500

throttle_max = 2000
throttle_min = 1000

steering_max = 2000
steering_min = 1000

step = 10

radio_on = False

try:
    while True:
        if keyboard.is_pressed("up"):
            if throttle < throttle_max:
                throttle += step

        elif keyboard.is_pressed("down"):
            if throttle > throttle_min:
                throttle -= step

        elif keyboard.is_pressed("left"):
            if steering > steering_min:
                steering -= step

        elif keyboard.is_pressed("right"):
            if steering < steering_max:
                steering += step

        try:
            msg = bus.recv(timeout=0.01)
            radio_on = True
        except can.CanError:
            if radio_on:
                time.sleep(1)
            radio_on = False

        if not radio_on:
            bus.send(servo.set_throttle_pulse_frame(throttle))
            bus.send(servo.set_steering_pulse_frame(steering))

except KeyboardInterrupt:
    bus.shutdown()
