import argparse
import enum
import struct
import sys
import threading

import can
import rclpy
import rover
import std_msgs.msg as msgtype
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy


@enum.unique
class BatteryPosition(enum.Enum):
    CONTROL_SYSTEM = "control_system"
    AD_SYSTEM = "ad_system"


class BatteryNode(Node):
    def __init__(self, interface, channel, bitrate):
        super().__init__("battery_node")

        self.declare_parameter("position", "control_system")

        position = self.get_parameter("position").get_parameter_value().string_value

        # Workaround for ROS Humble (python 3.10) which doesn't support StrEnum natively.
        valid_positions = [pos.value for pos in BatteryPosition]

        if position not in valid_positions:
            raise ValueError()

        self.position = BatteryPosition(position)

        self.get_logger().info(f"initializing {self.get_name()}")

        self.cell_voltages_topic = f"{self.get_name()}/cell_voltage_mV"
        self.cell_voltages_publisher = self.create_publisher(
            msgtype.UInt16MultiArray,
            self.cell_voltages_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.cell_voltages = []

        self.voltage_topic = f"{self.get_name()}/battery_output/voltage_mV"
        self.current_topic = f"{self.get_name()}/battery_output/current_mA"
        self.voltage_publisher = self.create_publisher(
            msgtype.UInt32,
            self.voltage_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.current_publisher = self.create_publisher(
            msgtype.UInt32,
            self.current_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.reg_output_voltage_topic = f"{self.get_name()}/regulated_output/voltage_mV"
        self.reg_output_current_topic = f"{self.get_name()}/regulated_output/current_mA"
        self.reg_output_voltage_publisher = self.create_publisher(
            msgtype.UInt32,
            self.reg_output_voltage_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.reg_output_current_publisher = self.create_publisher(
            msgtype.UInt32,
            self.reg_output_current_topic,
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

    def publish(self, msg):
        id = msg.arbitration_id

        if self.position == BatteryPosition.CONTROL_SYSTEM:
            if id == rover.Envelope.BATTERY_CELL_VOLTAGES:
                self.publish_cell_voltages(msg)
            if id == rover.Envelope.BATTERY_OUTPUT:
                self.publish_output(msg)
            if id == rover.Envelope.BATTERY_REGULATED_OUTPUT:
                self.publish_reg_output(msg)

        if self.position == BatteryPosition.AD_SYSTEM:
            if id == rover.Envelope.AD_BATTERY_CELL_VOLTAGES:
                self.publish_cell_voltages(msg)
            if id == rover.Envelope.AD_BATTERY_OUTPUT:
                self.publish_output(msg)
            if id == rover.Envelope.AD_BATTERY_REGULATED_OUTPUT:
                self.publish_reg_output(msg)

    def publish_cell_voltages(self, msg):
        # If first message received is for the last 3 cells, skip it
        # If something went wrong with the reception order, we also skip it
        if (msg.data[0] == 1 and len(self.cell_voltages) < 3) or (
            msg.data[0] == 0 and len(self.cell_voltages) > 0
        ):
            self.cell_voltages.clear()
            return

        self.cell_voltages.append(struct.unpack("H", msg.data[1:3])[0])
        self.cell_voltages.append(struct.unpack("H", msg.data[3:5])[0])
        self.cell_voltages.append(struct.unpack("H", msg.data[5:7])[0])

        if len(self.cell_voltages) >= 6:
            cell_voltage_msg = msgtype.UInt16MultiArray()
            cell_voltage_msg.data = self.cell_voltages
            self.cell_voltages_publisher.publish(cell_voltage_msg)
            self.get_logger().debug(
                f'Publishing {self.cell_voltages_topic}: "{cell_voltage_msg.data}"'
            )

            self.cell_voltages.clear()

    def publish_output(self, msg):
        voltage_msg = msgtype.UInt32()
        current_msg = msgtype.UInt32()

        voltage_msg.data = struct.unpack("I", msg.data[0:4])[0]
        current_msg.data = struct.unpack("I", msg.data[4:8])[0]

        self.voltage_publisher.publish(voltage_msg)
        self.current_publisher.publish(current_msg)
        self.get_logger().debug(
            f'Publishing {self.voltage_topic}: "{voltage_msg.data}"'
        )
        self.get_logger().debug(
            f'Publishing {self.current_topic}: "{current_msg.data}"'
        )

    def publish_reg_output(self, msg):
        voltage_msg = msgtype.UInt32()
        current_msg = msgtype.UInt32()

        voltage_msg.data = struct.unpack("I", msg.data[0:4])[0]
        current_msg.data = struct.unpack("I", msg.data[4:8])[0]

        self.reg_output_voltage_publisher.publish(voltage_msg)
        self.reg_output_current_publisher.publish(current_msg)
        self.get_logger().debug(
            f'Publishing {self.reg_output_voltage_topic}: "{voltage_msg.data}"'
        )
        self.get_logger().debug(
            f'Publishing {self.reg_output_current_topic}: "{current_msg.data}"'
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=125000)
    parsed_args, _ = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=args)

    node = BatteryNode(
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
