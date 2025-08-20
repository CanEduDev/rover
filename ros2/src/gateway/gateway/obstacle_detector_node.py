import argparse
import enum
import struct
import sys
import threading

import can
import rclpy
import rover
from gateway_msgs.msg import ObstacleDistance  # type: ignore
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


@enum.unique
class ObstacleDetectorPosition(enum.Enum):
    FRONT = "front"
    REAR = "rear"


class ObstacleDetectorNode(Node):
    def __init__(self, interface, channel, bitrate):
        super().__init__("obstacle_detector_node")

        self.declare_parameter("position", "front")

        position = self.get_parameter("position").get_parameter_value().string_value

        # Workaround for ROS Humble (python 3.10) which doesn't support StrEnum natively.
        valid_positions = [pos.value for pos in ObstacleDetectorPosition]

        if position not in valid_positions:
            raise ValueError()

        self.position = ObstacleDetectorPosition(position)

        self.get_logger().info(f"initializing {self.get_name()}")

        self.topic = f"{self.get_name()}/obstacle_distance"
        self.publisher = self.create_publisher(
            ObstacleDistance,
            self.topic,
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.init_can_bus(interface, channel, bitrate)
        self.get_logger().info("finished initialization")

    def destroy_node(self):
        super().destroy_node()
        self.can_bus.shutdown()

    def init_can_bus(self, interface, channel, bitrate):
        can_mask_11_bits = (1 << 11) - 1
        can_id = rover.Envelope.OBSTACLE_DETECTOR_FRONT_DISTANCE
        if self.position == ObstacleDetectorPosition.REAR:
            can_id = rover.Envelope.OBSTACLE_DETECTOR_REAR_DISTANCE

        self.can_bus = can.ThreadSafeBus(
            interface=interface, channel=channel, bitrate=bitrate
        )
        self.can_bus.set_filters([{"can_id": can_id, "can_mask": can_mask_11_bits}])

        threading.Thread(target=self.can_reader_task, daemon=True).start()

    def can_reader_task(self):
        for msg in self.can_bus:
            if not rclpy.ok():
                break

            self.publish(msg)

    def publish(self, msg):
        distance_msg = ObstacleDistance()
        distance_msg.distances_mm = [
            struct.unpack("H", msg.data[0:2])[0],
            struct.unpack("H", msg.data[2:4])[0],
            struct.unpack("H", msg.data[4:6])[0],
            struct.unpack("H", msg.data[6:8])[0],
        ]
        self.publisher.publish(distance_msg)
        self.get_logger().debug(
            f"Publishing {self.topic}: distances={distance_msg.distances_mm}mm"
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=125000)
    parsed_args, _ = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=args)

    node = ObstacleDetectorNode(
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
