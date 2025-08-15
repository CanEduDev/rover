import argparse
import math
import struct
import sys
import threading
import time

import can
import rclpy
import rclpy.parameter
import rover
from gateway_msgs.msg import CANStatus  # type: ignore
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


class ControllerNode(Node):
    def __init__(self, interface, channel, bitrate):
        super().__init__("controller_node")

        self.get_logger().info("initializing controller")

        # Controller cmd_vel subscription (for ROS control)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            ReliabilityPolicy.RELIABLE,
        )

        # Radio cmd_vel publisher (for radio override)
        self.radio_cmd_vel_publisher = self.create_publisher(
            Twist,
            "radio/cmd_vel",
            ReliabilityPolicy.BEST_EFFORT,
        )

        # Radio state tracking
        self._last_radio_throttle = 0.0
        self._last_radio_steering = 0.0

        self.radio_override_logged = False
        self.radio_override_lock = threading.Lock()
        self.last_radio_timestamp = 0
        self.throttle = 0
        self.steering_angle = 0
        self.control_freq_hz = 100
        self.control_timer = self.create_timer(
            1 / self.control_freq_hz, self.send_control_command
        )

        self.can_bus = can.ThreadSafeBus(
            interface=interface, channel=channel, bitrate=bitrate
        )

        # Add CAN enabled state
        self.can_enabled = True
        self.can_enabled_lock = threading.Lock()
        self.can_enabled_sub = self.create_subscription(
            CANStatus,
            "can_status",
            self.can_enabled_callback,
            ReliabilityPolicy.RELIABLE,
        )

        # Start CAN reader thread
        threading.Thread(target=self.can_reader_task, daemon=True).start()
        self.get_logger().info("finished initialization")

    def destroy_node(self):
        super().destroy_node()
        self.can_bus.shutdown()

    def can_enabled_callback(self, msg):
        with self.can_enabled_lock:
            self.can_enabled = msg.can_enabled

        # reset override state to avoid potential race condition with radio node on comm mode or action mode change
        self.refresh_radio_timestamp()
        self.radio_override_logged = True

    def can_reader_task(self):
        can_mask_11_bits = (1 << 11) - 1
        self.can_bus.set_filters(
            [
                {"can_id": rover.Envelope.STEERING, "can_mask": can_mask_11_bits},
                {"can_id": rover.Envelope.THROTTLE, "can_mask": can_mask_11_bits},
            ]
        )

        for msg in self.can_bus:
            if not rclpy.ok():
                break

            # Steering is always sent before throttle
            if msg.arbitration_id == rover.Envelope.STEERING:
                self.refresh_radio_timestamp()
                self.process_radio_steering(msg)

            elif msg.arbitration_id == rover.Envelope.THROTTLE:
                self.refresh_radio_timestamp()
                self.process_radio_throttle(msg)
                # Safely publish here as both steering and throttle have been updated
                self.publish_radio_cmd_vel()

    def process_radio_throttle(self, msg):
        # Process throttle from CAN message
        throttle_pulse = struct.unpack("H", msg.data[1:3])[0]
        # Normalize to [-1, 1] range
        throttle = round((throttle_pulse - 1500) / 500)
        if throttle > 1:
            throttle = 1
        if throttle < -1:
            throttle = -1

        self._last_radio_throttle = float(throttle)

    def process_radio_steering(self, msg):
        # Process steering from CAN message
        steering_data = float(struct.unpack("f", msg.data[1:5])[0])
        # Convert degrees to radians
        self._last_radio_steering = math.radians(steering_data)

    def publish_radio_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self._last_radio_throttle
        twist.angular.z = self._last_radio_steering
        self.radio_cmd_vel_publisher.publish(twist)

    def refresh_radio_timestamp(self):
        with self.radio_override_lock:
            self.last_radio_timestamp = time.monotonic()

    def get_radio_timestamp(self):
        with self.radio_override_lock:
            return self.last_radio_timestamp

    def stop_timer(self):
        self.control_timer.destroy()

    def start_timer(self):
        self.control_timer = self.create_timer(
            1 / self.control_freq_hz, self.send_control_command
        )

    # 1000-2000µs pulse width, 1500 is neutral
    def throttle_message(self):
        throttle_pulse = round(1500 + 500 * self.throttle)
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
        with self.can_enabled_lock:
            if not self.can_enabled:
                self.get_logger().debug(
                    "CAN communication is disabled, not sending control command."
                )
                return

        last_radio_timestamp = self.get_radio_timestamp()
        if last_radio_timestamp == 0 or time.monotonic() - last_radio_timestamp <= 0.1:
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
                "CAN error: control command not sent. Retrying in 1s"
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
        while time.time() - t < 2:
            time.sleep(0.01)
            self.send_control_command()

    def cmd_vel_callback(self, msg):
        # Twist linear.x is throttle (-1 to 1 range)
        throttle = msg.linear.x
        # Twist angular.z is steering angle, but convert radians to degrees
        steering_angle = math.degrees(msg.angular.z)

        self.get_logger().debug(
            f"Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z} -> throttle={throttle}, steering={steering_angle}"
        )

        switch_direction = False

        if self.throttle > 0 and throttle < 0:
            switch_direction = True
        if self.throttle < 0 and throttle > 0:
            switch_direction = True

        max_throttle = 1
        min_throttle = -1

        if throttle > max_throttle:
            self.get_logger().warn(
                f"Received invalid throttle value: {throttle}, limiting to {max_throttle}"
            )
            throttle = max_throttle

        if throttle < min_throttle:
            self.get_logger().warn(
                f"Received invalid throttle value: {throttle}, limiting to {min_throttle}"
            )
            throttle = min_throttle

        max_angle = 45
        min_angle = -45

        if steering_angle > max_angle:
            self.get_logger().warn(
                f"Received invalid steering angle: {steering_angle}, limiting to {max_angle} degrees"
            )
            steering_angle = max_angle

        if steering_angle < min_angle:
            self.get_logger().warn(
                f"Received invalid steering angle: {steering_angle}, limiting to {min_angle} degrees"
            )
            steering_angle = min_angle

        # Reversing directions needs special treatment
        if switch_direction:
            self.get_logger().info("Reversing direction")

            self.stop_timer()

            # Go to neutral
            self.manual_throttle_command(0)

            self.start_timer()

        self.throttle = throttle
        self.steering_angle = steering_angle


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=125000)
    parsed_args, _ = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=args)
    node = ControllerNode(
        interface=parsed_args.interface,
        channel=parsed_args.channel,
        bitrate=parsed_args.bitrate,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
