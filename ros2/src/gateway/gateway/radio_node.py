import argparse
import struct
import sys
import threading

import can
import rclpy
import rover
import std_msgs.msg as msgtype
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


class RadioNode(Node):
    def __init__(self, interface, channel, bitrate):
        super().__init__("radio_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        self.throttle_topic = f"{self.get_name()}/throttle"
        self.steering_topic = f"{self.get_name()}/steering"
        self.throttle_publisher = self.create_publisher(
            msgtype.Float32,
            self.throttle_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.steering_publisher = self.create_publisher(
            msgtype.Float32,
            self.steering_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )

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
        throttle_msg = msgtype.Float32()

        throttle_pulse = struct.unpack("H", msg.data[1:3])[0]
        throttle = round((throttle_pulse - 1500) / 5)
        if throttle > 100:
            throttle = 100
        if throttle < -100:
            throttle = -100

        throttle_msg.data = float(throttle)
        self.get_logger().debug(
            f'Publishing {self.throttle_topic}: "{throttle_msg.data}"'
        )
        self.throttle_publisher.publish(throttle_msg)

    def publish_steering(self, msg):
        steering_msg = msgtype.Float32()
        steering_msg.data = struct.unpack("f", msg.data[1:5])[0]
        self.get_logger().debug(
            f'Publishing {self.steering_topic}: "{steering_msg.data}"'
        )
        self.steering_publisher.publish(steering_msg)

    def publish(self, msg):
        id = msg.arbitration_id
        if id == rover.Envelope.THROTTLE:
            self.publish_throttle(msg)
        elif id == rover.Envelope.STEERING:
            self.publish_steering(msg)


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
