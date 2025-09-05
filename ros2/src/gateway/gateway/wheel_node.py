import argparse
import enum
import struct
import sys
import threading

import can
import rclpy
import rover
from gateway_msgs.msg import CANStatus, ReportFrequency, WheelStatus  # type: ignore
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


@enum.unique
class WheelPosition(enum.Enum):
    FRONT_LEFT = "front_left"
    FRONT_RIGHT = "front_right"
    REAR_LEFT = "rear_left"
    REAR_RIGHT = "rear_right"


class WheelNode(Node):
    def __init__(self, interface, channel, bitrate):
        super().__init__("wheel_node")

        self.declare_parameter("position", "front_left")

        position = self.get_parameter("position").get_parameter_value().string_value

        # Workaround for ROS Humble (python 3.10) which doesn't support StrEnum natively.
        valid_positions = [pos.value for pos in WheelPosition]

        if position not in valid_positions:
            raise ValueError()

        self.position = WheelPosition(position)

        self.get_logger().info(f"initializing {self.get_name()}")

        self.wheel_status_topic = f"{self.get_name()}/wheel_status"
        self.wheel_status_publisher = self.create_publisher(
            WheelStatus, self.wheel_status_topic, ReliabilityPolicy.BEST_EFFORT
        )

        self.report_freq_topic = f"{self.get_name()}/report_frequency_hz"
        self.report_freq_subscriber = self.create_subscription(
            ReportFrequency,
            self.report_freq_topic,
            self.report_freq_callback,
            ReliabilityPolicy.RELIABLE,
        )

        self.init_can_bus(interface, channel, bitrate)
        self.get_logger().info("finished initialization")

    def destroy_node(self):
        super().destroy_node()
        self.can_bus.shutdown()

    def init_can_bus(self, interface, channel, bitrate):
        can_mask_11_bits = (1 << 11) - 1
        can_id = rover.Envelope.WHEEL_FRONT_LEFT_SPEED
        self.report_freq_id = rover.Envelope.WHEEL_FRONT_LEFT_REPORT_FREQUENCY
        if self.position == WheelPosition.FRONT_RIGHT:
            can_id = rover.Envelope.WHEEL_FRONT_RIGHT_SPEED
            self.report_freq_id = rover.Envelope.WHEEL_FRONT_RIGHT_REPORT_FREQUENCY
        elif self.position == WheelPosition.REAR_LEFT:
            can_id = rover.Envelope.WHEEL_REAR_LEFT_SPEED
            self.report_freq_id = rover.Envelope.WHEEL_REAR_LEFT_REPORT_FREQUENCY
        elif self.position == WheelPosition.REAR_RIGHT:
            can_id = rover.Envelope.WHEEL_REAR_RIGHT_SPEED
            self.report_freq_id = rover.Envelope.WHEEL_REAR_RIGHT_REPORT_FREQUENCY

        can_filters = [{"can_id": can_id, "can_mask": can_mask_11_bits}]

        self.can_bus = can.ThreadSafeBus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            can_filters=can_filters,
        )

        # Flush receive buffer because sometimes messages pass through before filter is applied.
        while self.can_bus.recv(timeout=0) is not None:
            pass

        # Add CAN enabled state
        self.can_enabled = True
        self.can_enabled_lock = threading.Lock()
        self.can_enabled_sub = self.create_subscription(
            CANStatus, "can_status", self.can_enabled_callback, 10
        )

        threading.Thread(target=self.can_reader_task, daemon=True).start()

    def can_enabled_callback(self, msg):
        with self.can_enabled_lock:
            self.can_enabled = msg.can_enabled

    def can_reader_task(self):
        for msg in self.can_bus:
            if not rclpy.ok():
                break

            self.publish(msg)

    def publish(self, msg):
        wheel_status = WheelStatus()
        wheel_status.rpm = struct.unpack("f", msg.data[0:4])[0]
        wheel_status.speed_kph = struct.unpack("f", msg.data[4:8])[0]
        self.wheel_status_publisher.publish(wheel_status)
        self.get_logger().debug(
            f"Publishing {self.wheel_status_topic}: rpm={wheel_status.rpm}, speed={wheel_status.speed_kph}kph"
        )

    def report_freq_callback(self, msg):
        max_freq = 100
        min_freq = 1

        report_freq_hz = msg.frequency_hz
        self.get_logger().info("Received report frequency change request")

        if report_freq_hz > max_freq:
            self.get_logger().warn(
                f"Received invalid frequency value: {msg.frequency_hz}, limiting to {max_freq}"
            )
            report_freq_hz = max_freq

        if report_freq_hz < min_freq:
            self.get_logger().warn(
                f"Received invalid frequency value: {msg.frequency_hz}, limiting to {min_freq}"
            )
            report_freq_hz = max_freq

        self.report_freq_hz = report_freq_hz

        with self.can_enabled_lock:
            if not self.can_enabled:
                self.get_logger().info(
                    "CAN communication is disabled, not sending config command."
                )
                return

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
        report_freq_ms = round(1000 / self.report_freq_hz)

        return can.Message(
            arbitration_id=self.report_freq_id,
            data=list(struct.pack("H", report_freq_ms)),
            is_extended_id=False,
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=125000)
    parsed_args, _ = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=args)
    node = WheelNode(
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
