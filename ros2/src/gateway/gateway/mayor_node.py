import argparse
import sys
import threading
import time

import can
import rclpy
import rover
from gateway_msgs.msg import CANStatus  # type: ignore
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy

# TODO: add timer to publish can_enabled regularly for boot up and node restart purposes.
# Nodes should default to disabled until this node is started.


class MayorNode(Node):
    def __init__(self, interface, channel, bitrate):
        super().__init__("mayor_node")
        self.can_enabled = True
        self.can_enabled_lock = threading.Lock()

        # Publisher for CAN enabled state
        self.can_enabled_pub = self.create_publisher(
            CANStatus, "can_status", ReliabilityPolicy.RELIABLE
        )
        self.publish_can_enabled()

        self.can_bus = can.ThreadSafeBus(
            interface=interface, channel=channel, bitrate=bitrate
        )

        self.kp_id = 0

        # Start CAN reader thread
        threading.Thread(target=self.can_reader_task, daemon=True).start()
        self.get_logger().info("finished initialization")

    def destroy_node(self):
        super().destroy_node()
        self.can_bus.shutdown()

    def disable_can(self):
        with self.can_enabled_lock:
            self.can_enabled = False
            self.publish_can_enabled()

        self.get_logger().info("Disabled CAN due to king's page command.")

    def enable_can(self):
        with self.can_enabled_lock:
            self.can_enabled = True
            self.publish_can_enabled()

        self.get_logger().info("Enabled CAN due to king's page command.")

    def publish_can_enabled(self):
        msg = CANStatus()
        msg.can_enabled = self.can_enabled
        msg.timestamp = int(time.time() * 1000000)  # Convert to microseconds
        self.can_enabled_pub.publish(msg)
        self.get_logger().debug(
            f"Published can_status: enabled={msg.can_enabled}, timestamp={msg.timestamp}"
        )

    def can_reader_task(self):
        for msg in self.can_bus:
            if not rclpy.ok():
                break

            if not self.is_kings_page(msg):
                continue

            city = msg.data[0]
            if city != 0 and city != rover.City.AD_ROS_GATEWAY:
                continue

            kings_page = msg.data[1]
            if kings_page != 0:
                continue

            action_mode = msg.data[2]
            comm_mode = msg.data[3]
            if (
                action_mode == rover.ActionMode.FREEZE
                or comm_mode == rover.CommMode.LISTEN_ONLY
                or comm_mode == rover.CommMode.SILENT
            ):
                self.disable_can()
            elif (
                action_mode == rover.ActionMode.RUN
                or comm_mode == rover.CommMode.COMMUNICATE
            ):
                self.enable_can()

    def is_kings_page(self, msg):
        return msg.arbitration_id == self.kp_id and len(msg.data) == 8


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=125000)
    parsed_args, _ = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=args)
    node = MayorNode(
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
