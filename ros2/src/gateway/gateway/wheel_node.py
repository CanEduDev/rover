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

        # Workaround for ROS Humble (python 3.10) which doesn't support StrEnum natively.
        valid_positions = [pos.value for pos in WheelPosition]

        if position not in valid_positions:
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

        self.report_freq_topic = f"{self.get_name()}/report_frequency_hz"
        self.report_freq_subscriber = self.create_subscription(
            msgtype.UInt32,
            self.report_freq_topic,
            self.report_freq_callback,
            ReliabilityPolicy.RELIABLE,
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

    def report_freq_callback(self, msg):
        max_freq = 100
        min_freq = 1

        report_freq_hz = msg.data
        self.get_logger().info("Received report frequency change request")

        if report_freq_hz > max_freq:
            self.get_logger().warn(
                f"Received invalid frequency value: {msg.data}, limiting to {max_freq}"
            )
            report_freq_hz = max_freq

        if report_freq_hz < min_freq:
            self.get_logger().warn(
                f"Received invalid frequency value: {msg.data}, limiting to {min_freq}"
            )
            report_freq_hz = max_freq

        self.report_freq_hz = report_freq_hz

        try:
            self.can_bus.send(self.set_report_period_message())
            self.get_logger().info(
                f"sent CAN configuration command (report period: {self.report_freq_hz} hz"
            )

        except can.exceptions.CanOperationError:
            self.get_logger().error(
                "CAN error: configuration command not sent. Will not retry transmission."
            )
            self.get_logger().info(
                "Potential causes: Rover offline, broken CAN connection, CAN Tx buffer overflow, CAN error frame or invalid bitrate setting"
            )
            self.can_bus.flush_tx_buffer()

    def set_report_period_message(self):
        if self.position == WheelPosition.FRONT_LEFT:
            envelope = rover.Envelope.WHEEL_FRONT_LEFT_REPORT_FREQUENCY
        elif self.position == WheelPosition.FRONT_RIGHT:
            envelope = rover.Envelope.WHEEL_FRONT_RIGHT_REPORT_FREQUENCY
        elif self.position == WheelPosition.REAR_LEFT:
            envelope = rover.Envelope.WHEEL_REAR_LEFT_REPORT_FREQUENCY
        else:
            envelope = rover.Envelope.WHEEL_REAR_RIGHT_REPORT_FREQUENCY

        report_freq_ms = round(1000 / self.report_freq_hz)

        return can.Message(
            arbitration_id=envelope,
            data=list(struct.pack("H", report_freq_ms)),
            is_extended_id=False,
        )


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
