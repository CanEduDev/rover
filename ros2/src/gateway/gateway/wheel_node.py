import enum
import struct

import threading
import can
import rclpy
import std_msgs.msg as msgtype
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy

import rover


@enum.unique
class WheelPosition(enum.Enum):
    FRONT_LEFT = "front_left"
    FRONT_RIGHT = "front_right"
    REAR_LEFT = "rear_left"
    REAR_RIGHT = "rear_right"


class WheelNode(Node):
    def __init__(self):
        super().__init__("wheel_node")

        self.declare_parameter("position", "front_left")

        position = self.get_parameter("position").get_parameter_value().string_value

        if position not in WheelPosition:
            raise ValueError()

        self.position = WheelPosition(position)

        self.get_logger().info(f"initializing {self.get_name()}")

        self.rpm_topic = f"{self.get_name()}/rpm"
        self.speed_topic = f"{self.get_name()}/speed_kph"

        self.rpm_publisher = self.create_publisher(
            msgtype.Float32, self.rpm_topic, ReliabilityPolicy.BEST_EFFORT
        )
        self.speed_publisher = self.create_publisher(
            msgtype.Float32,
            self.speed_topic,
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

    def publish_rpm(self, msg):
        rpm_msg = msgtype.Float32()
        rpm_msg.data = struct.unpack("f", msg.data[0:4])[0]
        self.rpm_publisher.publish(rpm_msg)
        self.get_logger().debug(f'Publishing {self.rpm_topic}: "{rpm_msg.data}"')

    def publish_speed(self, msg):
        speed_msg = msgtype.Float32()
        speed_msg.data = struct.unpack("f", msg.data[4:8])[0]
        self.get_logger().debug(f'Publishing {self.speed_topic}: "{speed_msg.data}"')
        self.speed_publisher.publish(speed_msg)

    def publish(self, msg):
        id = msg.arbitration_id

        if (
            (
                self.position == WheelPosition.FRONT_LEFT
                and id == rover.Envelope.WHEEL_FRONT_LEFT_SPEED
            )
            or (
                self.position == WheelPosition.FRONT_RIGHT
                and id == rover.Envelope.WHEEL_FRONT_RIGHT_SPEED
            )
            or (
                self.position == WheelPosition.REAR_LEFT
                and id == rover.Envelope.WHEEL_REAR_LEFT_SPEED
            )
            or (
                self.position == WheelPosition.REAR_RIGHT
                and id == rover.Envelope.WHEEL_REAR_RIGHT_SPEED
            )
        ):
            self.publish_rpm(msg)
            self.publish_speed(msg)


def main(args=None):
    rclpy.init(args=args)

    node = WheelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
