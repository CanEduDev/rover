import struct
import threading
import time

import can
import rclpy
import rover
import std_msgs.msg as msgtype
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.get_logger().info("initializing controller")

        self.throttle_sub = self.create_subscription(
            msgtype.Float32,
            "throttle",
            self.throttle_callback,
            ReliabilityPolicy.RELIABLE,
        )

        self.steering_sub = self.create_subscription(
            msgtype.Float32,
            "steering",
            self.steering_callback,
            ReliabilityPolicy.RELIABLE,
        )

        self.radio_override_logged = False
        self.last_radio_timestamp = 0
        self.throttle = 1500
        self.steering_angle = 0
        self.control_freq_hz = 100
        self.control_timer = self.create_timer(
            1 / self.control_freq_hz, self.send_control_command
        )

        self.can_bus = can.ThreadSafeBus(
            interface="socketcan", channel="can0", bitrate=125_000
        )
        self.can_reader_thread = threading.Thread(target=self.can_reader_task)
        # Daemon thread will exit when the main program ends
        self.can_reader_thread.daemon = True
        self.can_reader_thread.start()

        self.get_logger().info("finished initialization")

    def destroy_node(self):
        self.can_reader_thread.join()
        self.can_bus.shutdown()
        super().destroy_node()

    def can_reader_task(self):
        for msg in self.can_bus:
            if (
                msg.arbitration_id == rover.Envelope.STEERING
                or msg.arbitration_id == rover.Envelope.THROTTLE
            ):
                self.refresh_radio_timestamp(msg.timestamp)

            if not rclpy.ok():
                break

    def refresh_radio_timestamp(self, timestamp):
        self.last_radio_timestamp = timestamp

    def stop_timer(self):
        self.control_timer.destroy()

    def start_timer(self):
        self.control_timer = self.create_timer(
            1 / self.control_freq_hz, self.send_control_command
        )

    # 1000-2000µs pulse width, 1500 is neutral
    def throttle_message(self):
        throttle_pulse = round(1500 + 5 * self.throttle)
        return can.Message(
            arbitration_id=rover.Envelope.THROTTLE,
            data=[0] + list(struct.pack("I", throttle_pulse)),
            is_extended_id=False,
        )

    def steering_message(self):
        return can.Message(
            arbitration_id=rover.Envelope.STEERING,
            data=[1] + list(struct.pack("f", self.steering_angle)),
            is_extended_id=False,
        )

    def send_control_command(self):
        if (
            self.last_radio_timestamp == 0
            or time.time() - self.last_radio_timestamp <= 0.1
        ):
            if not self.radio_override_logged:
                self.radio_override_logged = True
                self.get_logger().info(
                    "Safety override activated. Will not send control commands"
                )
            return

        if self.radio_override_logged:
            self.radio_override_logged = False
            self.get_logger().info("Safety override inactivated")

        try:
            self.can_bus.send(self.throttle_message())
            self.can_bus.send(self.steering_message())
            self.get_logger().debug(
                f"sent CAN control command (throttle: {self.throttle}, steering: {self.steering_angle})"
            )
        except can.exceptions.CanOperationError:
            self.get_logger().error(
                "CAN error: steering command not sent. Retrying in 1s"
            )
            self.get_logger().info(
                "Potential causes: Rover offline, broken CAN connection, CAN Tx buffer overflow, CAN error frame or invalid bitrate setting"
            )
            self.can_bus.flush_tx_buffer()

            self.stop_timer()
            time.sleep(1)
            self.start_timer()

    def manual_throttle_command(self, throttle):
        self.throttle = throttle

        t = time.time()
        while time.time() - t < 1:
            time.sleep(0.01)
            self.send_control_command()

    def throttle_callback(self, msg):
        # Negative values are reverse, positive are forward
        throttle = msg.data
        self.get_logger().debug(f"Received throttle: {throttle}")

        switch_reverse = False
        switch_forward = False

        if self.throttle > 0 and throttle < 0:
            switch_reverse = True
        if self.throttle < 0 and throttle > 0:
            switch_forward = True

        max_throttle = 100
        min_throttle = -100

        if throttle > max_throttle:
            self.get_logger().warn(
                f"Received invalid throttle value: {msg.data}, limiting to {max_throttle}"
            )
            throttle = max_throttle

        if throttle < min_throttle:
            self.get_logger().warn(
                f"Received invalid throttle value: {msg.data}, limiting to {min_throttle}"
            )
            throttle = min_throttle

        # Reversing directions needs special treatment
        if switch_reverse or switch_forward:
            self.get_logger().info("Reversing direction")

            self.stop_timer()

            # Braking required when switching from forward to reverse
            if switch_reverse:
                self.manual_throttle_command(-50)

            # Add extra neutral command so timing is equal when switching from reverse to forward
            if switch_forward:
                self.manual_throttle_command(0)

            self.manual_throttle_command(0)

            self.start_timer()

        self.throttle = throttle

    def steering_callback(self, msg):
        max_angle = 45
        min_angle = -45
        steering_angle = msg.data
        self.get_logger().debug(f"Received steering angle: {steering_angle} degrees")

        if steering_angle > max_angle:
            self.get_logger().warn(
                f"Received invalid steering angle: {msg.data}, limiting to {max_angle} degrees"
            )
            steering_angle = max_angle

        if steering_angle < min_angle:
            self.get_logger().warn(
                f"Received invalid steering angle: {msg.data}, limiting to {min_angle} degrees"
            )
            steering_angle = min_angle

        self.steering_angle = steering_angle


def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
