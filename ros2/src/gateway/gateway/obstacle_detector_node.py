import enum
import struct

import threading
import can
import rclpy
import rover
import std_msgs.msg as msgtype
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


@enum.unique
class ObstacleDetectorPosition(enum.Enum):
    FRONT = "front"
    REAR = "rear"


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector_node")

        self.declare_parameter("position", "front")

        position = self.get_parameter("position").get_parameter_value().string_value

        # Workaround for ROS Humble (python 3.10) which doesn't support StrEnum natively.
        valid_positions = [pos.value for pos in ObstacleDetectorPosition]

        if position not in valid_positions:
            raise ValueError()

        self.position = ObstacleDetectorPosition(position)

        self.get_logger().info(f"initializing {self.get_name()}")

        self.topic = f"{self.get_name()}/distance_mm"
        self.publisher = self.create_publisher(
            msgtype.UInt16MultiArray,
            self.topic,
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.can_bus = can.ThreadSafeBus(
            interface="socketcan", channel="can0", bitrate=125_000
        )
        self.can_reader_thread = threading.Thread(target=self.can_reader_task)
        # Daemon thread will exit when the main program ends
        self.can_reader_thread.daemon = True
        self.can_reader_thread.start()

        self.get_logger().info("finished intialization")

    def destroy_node(self):
        self.can_reader_thread.join()
        self.can_bus.shutdown()
        super().destroy_node()

    def can_reader_task(self):
        for msg in self.can_bus:
            self.publish(msg)

            if not rclpy.ok():
                break

    def publish(self, msg):
        id = msg.arbitration_id
        if (
            self.position == ObstacleDetectorPosition.FRONT
            and id == rover.Envelope.OBSTACLE_DETECTOR_FRONT_DISTANCE
        ) or (
            self.position == ObstacleDetectorPosition.REAR
            and id == rover.Envelope.OBSTACLE_DETECTOR_REAR_DISTANCE
        ):
            distance_msg = msgtype.UInt16MultiArray()
            distance_msg.data = [
                struct.unpack("H", msg.data[0:2])[0],
                struct.unpack("H", msg.data[2:4])[0],
                struct.unpack("H", msg.data[4:6])[0],
                struct.unpack("H", msg.data[6:8])[0],
            ]
            self.publisher.publish(distance_msg)
            self.get_logger().debug(f'Publishing {self.topic}: "{distance_msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
