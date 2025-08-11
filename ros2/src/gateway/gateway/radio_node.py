import argparse
import math
import struct
import sys
import threading

import can
import rclpy
import rover
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


class RadioNode(Node):
    def __init__(self, interface, channel, bitrate):
        super().__init__("radio_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            ReliabilityPolicy.BEST_EFFORT,
        )
        self._last_throttle = 0.0
        self._last_steering = 0.0

        self.can_bus = can.ThreadSafeBus(
            interface=interface, channel=channel, bitrate=bitrate
        )

        threading.Thread(target=self.can_reader_task, daemon=True).start()
        self.get_logger().info("finished intialization")

    def destroy_node(self):
        super().destroy_node()
        self.can_bus.shutdown()

    def can_reader_task(self):
        for msg in self.can_bus:
            if not rclpy.ok():
                break

            self.publish(msg)

    def publish_throttle(self, msg):
        # Process throttle from CAN message
        throttle_pulse = struct.unpack("H", msg.data[1:3])[0]
        throttle = round((throttle_pulse - 1500) / 5)
        if throttle > 100:
            throttle = 100
        if throttle < -100:
            throttle = -100

        self._last_throttle = throttle / 100.0  # Normalize to [-1, 1] for cmd_vel
        self.publish_cmd_vel()

    def publish_steering(self, msg):
        # Process steering from CAN message
        steering_data = float(struct.unpack("f", msg.data[1:5])[0])
        self._last_steering = steering_data
        self.publish_cmd_vel()

    def publish(self, msg):
        id = msg.arbitration_id
        if id == rover.Envelope.THROTTLE:
            self.publish_throttle(msg)
        elif id == rover.Envelope.STEERING:
            self.publish_steering(msg)

    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self._last_throttle  # normalized [-1, 1]
        twist.angular.z = math.radians(
            self._last_steering
        )  # convert degrees to radians
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=125000)
    parsed_args, _ = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=args)

    node = RadioNode(
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
