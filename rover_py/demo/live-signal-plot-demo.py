import argparse
import threading
import tkinter as tk

import matplotlib
import numpy as np

matplotlib.use("TkAgg")
import cantools
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from rover.can_interface import add_can_args, create_bus_from_args

# --- Configurable parameters ---
MAX_POINTS = 500


# --- Data storage using numpy arrays ---
class CircularBuffer:
    def __init__(self, maxlen, dtype=float):
        self.maxlen = maxlen
        self.buffer = np.zeros(maxlen, dtype=dtype)
        self.timestamps = np.zeros(maxlen, dtype=float)
        self.index = 0
        self.is_full = False

    def append(self, value, timestamp=None):
        self.buffer[self.index] = value
        if timestamp is not None:
            self.timestamps[self.index] = timestamp
        self.index = (self.index + 1) % self.maxlen
        if self.index == 0:
            self.is_full = True

    def get_data(self):
        if self.is_full:
            return np.roll(self.buffer, -self.index), np.roll(
                self.timestamps, -self.index
            )
        # Only return data up to the current index
        return self.buffer[: self.index], self.timestamps[: self.index]


class MultiCircularBuffer:
    def __init__(self, maxlen, num_buffers, dtype=float):
        self.buffers = [CircularBuffer(maxlen, dtype) for _ in range(num_buffers)]

    def append(self, index, value, timestamp=None):
        self.buffers[index].append(value, timestamp)

    def get_data(self, index):
        return self.buffers[index].get_data()


# Initialize data storage
data = {
    "battery_cell_voltages": MultiCircularBuffer(MAX_POINTS, 6),
    "servo_current": CircularBuffer(MAX_POINTS),
    "throttle": CircularBuffer(MAX_POINTS),
    "steering_angle": CircularBuffer(MAX_POINTS),
    "wheel_speeds": MultiCircularBuffer(MAX_POINTS, 4),
    "battery_output_current": CircularBuffer(MAX_POINTS),
}

# --- Message and signal names as in DBC ---
MSG_NAMES = {
    "BATTERY_CELL_VOLTAGES": "BATTERY_CELL_VOLTAGES",
    "SERVO_CURRENT": "SERVO_CURRENT",
    "THROTTLE": "THROTTLE",
    "STEERING": "STEERING",
    "WHEEL_FRONT_LEFT_SPEED": "WHEEL_FRONT_LEFT_SPEED",
    "WHEEL_FRONT_RIGHT_SPEED": "WHEEL_FRONT_RIGHT_SPEED",
    "WHEEL_REAR_LEFT_SPEED": "WHEEL_REAR_LEFT_SPEED",
    "WHEEL_REAR_RIGHT_SPEED": "WHEEL_REAR_RIGHT_SPEED",
    "BATTERY_OUTPUT": "BATTERY_OUTPUT",
}


def can_receiver(bus, db):
    start_time = None
    frame_id_to_msg = {msg.frame_id: msg for msg in db.messages}
    wheel_names = [
        MSG_NAMES["WHEEL_FRONT_LEFT_SPEED"],
        MSG_NAMES["WHEEL_FRONT_RIGHT_SPEED"],
        MSG_NAMES["WHEEL_REAR_LEFT_SPEED"],
        MSG_NAMES["WHEEL_REAR_RIGHT_SPEED"],
    ]

    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue
        if msg.arbitration_id not in frame_id_to_msg:
            continue
        dbc_msg = frame_id_to_msg[msg.arbitration_id]
        try:
            decoded = dbc_msg.decode(msg.data)
        except Exception:
            continue

        t = msg.timestamp
        if start_time is None:
            start_time = t
        t = t - start_time  # Convert to seconds since start

        if dbc_msg.name == MSG_NAMES["BATTERY_CELL_VOLTAGES"]:
            if decoded["CELLS"] == "CELLS_1_TO_3":
                data["battery_cell_voltages"].append(0, decoded["CELL_1_VOLTAGE"], t)
                data["battery_cell_voltages"].append(1, decoded["CELL_2_VOLTAGE"], t)
                data["battery_cell_voltages"].append(2, decoded["CELL_3_VOLTAGE"], t)
            elif decoded["CELLS"] == "CELLS_4_TO_6":
                data["battery_cell_voltages"].append(3, decoded["CELL_4_VOLTAGE"], t)
                data["battery_cell_voltages"].append(4, decoded["CELL_5_VOLTAGE"], t)
                data["battery_cell_voltages"].append(5, decoded["CELL_6_VOLTAGE"], t)

        elif dbc_msg.name == MSG_NAMES["SERVO_CURRENT"]:
            data["servo_current"].append(decoded.get("SERVO_CURRENT", 0), t)
        elif dbc_msg.name == MSG_NAMES["THROTTLE"]:
            data["throttle"].append(decoded.get("THROTTLE_PULSE_WIDTH", 0) * 1000, t)
        elif dbc_msg.name == MSG_NAMES["STEERING"]:
            data["steering_angle"].append(decoded.get("STEERING_ANGLE", 0), t)
        elif dbc_msg.name in wheel_names:
            idx = wheel_names.index(dbc_msg.name)
            data["wheel_speeds"].append(idx, decoded.get("SPEED", 0), t)
        elif dbc_msg.name == MSG_NAMES["BATTERY_OUTPUT"]:
            data["battery_output_current"].append(
                decoded.get("BATTERY_OUTPUT_CURRENT", 0), t
            )


def main():
    parser = argparse.ArgumentParser(description="Live CAN signal plot demo")
    add_can_args(parser)
    parser.add_argument(
        "--dbc", default="rover.dbc", help="DBC file path (default: rover.dbc)"
    )
    args = parser.parse_args()

    db = cantools.db.load_file(args.dbc)
    bus = create_bus_from_args(args)

    threading.Thread(target=can_receiver, args=(bus, db), daemon=True).start()

    root = tk.Tk()
    root.title("Live CAN Signal Plot Demo")

    fig, axs = plt.subplots(3, 2, figsize=(10, 8))
    fig.tight_layout(pad=3.0, h_pad=6.0)
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def animate(_):
        artists = []

        # Throttle
        axs[0, 0].cla()
        x, t = data["throttle"].get_data()
        if len(x) > 0:
            (line,) = axs[0, 0].plot(t, x, label="Throttle")
            artists.append(line)
        axs[0, 0].set_title("Throttle Pulse Width (µs)")
        axs[0, 0].set_ylabel("µs")
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].legend(loc="upper right")

        # Steering angle
        axs[0, 1].cla()
        x, t = data["steering_angle"].get_data()
        if len(x) > 0:
            (line,) = axs[0, 1].plot(t, x, label="Steering Angle")
            artists.append(line)
        axs[0, 1].set_title("Steering Angle (deg)")
        axs[0, 1].set_ylabel("deg")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].legend(loc="upper right")

        # Battery cell voltages
        axs[1, 0].cla()
        for i in range(6):
            y, t = data["battery_cell_voltages"].get_data(i)
            if len(y) > 0:
                (line,) = axs[1, 0].plot(t, y, label=f"Cell {i + 1}")
                artists.append(line)
        axs[1, 0].set_title("Battery Cell Voltages (mV)")
        axs[1, 0].set_ylabel("mV")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].legend(loc="upper right")

        # Wheel speeds
        axs[1, 1].cla()
        wheel_labels = ["FL", "FR", "RL", "RR"]
        for i in range(4):
            y, t = data["wheel_speeds"].get_data(i)
            if len(y) > 0:
                (line,) = axs[1, 1].plot(t, y, label=wheel_labels[i])
                artists.append(line)
        axs[1, 1].set_title("Wheel Speeds (km/h)")
        axs[1, 1].set_ylabel("km/h")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].legend(loc="upper right")

        # Battery Output Current
        axs[2, 0].cla()
        x, t = data["battery_output_current"].get_data()
        if len(x) > 0:
            (line,) = axs[2, 0].plot(t, x, label="Battery Output Current")
            artists.append(line)
        axs[2, 0].set_title("Battery Output Current (mA)")
        axs[2, 0].set_ylabel("mA")
        axs[2, 0].set_xlabel("Time (s)")
        axs[2, 0].legend(loc="upper right")

        # Servo current
        axs[2, 1].cla()
        x, t = data["servo_current"].get_data()
        if len(x) > 0:
            (line,) = axs[2, 1].plot(t, x, label="Servo Current")
            artists.append(line)
        axs[2, 1].set_title("Servo Current (mA)")
        axs[2, 1].set_ylabel("mA")
        axs[2, 1].set_xlabel("Time (s)")
        axs[2, 1].legend(loc="upper right")

        return artists

    _ani = FuncAnimation(
        fig, animate, interval=(1000 / 60), cache_frame_data=False
    )  # 60 fps

    def on_close():
        root.quit()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
