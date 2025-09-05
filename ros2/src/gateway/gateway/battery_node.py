import argparse
import enum
import struct
import sys
import threading

import can
import rclpy
import rover
from gateway_msgs.msg import BatteryStatus  # type: ignore
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

        # Cell voltages topic
        self.cell_voltages_topic = f"{self.get_name()}/cell_voltages"
        self.cell_voltages_publisher = self.create_publisher(
            BatteryStatus,
            self.cell_voltages_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.cell_voltages = []

        # Output voltage/current topic
        self.output_topic = f"{self.get_name()}/output"
        self.output_publisher = self.create_publisher(
            BatteryStatus,
            self.output_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )

        # Regulated output voltage/current topic
        self.regulated_topic = f"{self.get_name()}/regulated"
        self.regulated_publisher = self.create_publisher(
            BatteryStatus,
            self.regulated_topic,
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.init_can_bus(interface, channel, bitrate)
        self.get_logger().info("finished initialization")

    def destroy_node(self):
        super().destroy_node()
        self.can_bus.shutdown()

    def init_can_bus(self, interface, channel, bitrate):
        self.cell_voltages_id = rover.Envelope.BATTERY_CELL_VOLTAGES
        self.output_id = rover.Envelope.BATTERY_OUTPUT
        self.regulated_output_id = rover.Envelope.BATTERY_REGULATED_OUTPUT

        if self.position == BatteryPosition.AD_SYSTEM:
            self.cell_voltages_id = rover.Envelope.AD_BATTERY_CELL_VOLTAGES
            self.output_id = rover.Envelope.AD_BATTERY_OUTPUT
            self.regulated_output_id = rover.Envelope.AD_BATTERY_REGULATED_OUTPUT

        can_mask_11_bits = (1 << 11) - 1
        can_filters = [
            {"can_id": self.cell_voltages_id, "can_mask": can_mask_11_bits},
            {"can_id": self.output_id, "can_mask": can_mask_11_bits},
            {"can_id": self.regulated_output_id, "can_mask": can_mask_11_bits},
        ]

        self.can_bus = can.ThreadSafeBus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            can_filters=can_filters,
        )

        # Flush receive buffer because sometimes messages pass through before filter is applied.
        while self.can_bus.recv(timeout=0) is not None:
            pass

        threading.Thread(target=self.can_reader_task, daemon=True).start()

    def can_reader_task(self):
        for msg in self.can_bus:
            if not rclpy.ok():
                break

            self.publish(msg)

    def publish(self, msg):
        id = msg.arbitration_id

        if id == self.cell_voltages_id:
            self.publish_cell_voltages(msg)
        elif id == self.output_id:
            self.publish_output(msg)
        elif id == self.regulated_output_id:
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
            battery_status = BatteryStatus()
            # Store cell voltages in millivolts as received
            battery_status.cell_voltages_mv = self.cell_voltages.copy()
            self.cell_voltages_publisher.publish(battery_status)
            self.get_logger().debug(
                f"Publishing {self.cell_voltages_topic}: cell_voltages_mv={battery_status.cell_voltages_mv}mV"
            )
            self.cell_voltages.clear()

    def publish_output(self, msg):
        battery_status = BatteryStatus()
        # Store voltage and current in millivolts/milliamps as received
        battery_status.output_voltage_mv = struct.unpack("I", msg.data[0:4])[0]
        battery_status.output_current_ma = struct.unpack("I", msg.data[4:8])[0]
        self.output_publisher.publish(battery_status)
        self.get_logger().debug(
            f"Publishing {self.output_topic}: output_voltage_mv={battery_status.output_voltage_mv}mV, output_current_ma={battery_status.output_current_ma}mA"
        )

    def publish_reg_output(self, msg):
        battery_status = BatteryStatus()
        # Store voltage and current in millivolts/milliamps as received
        battery_status.regulated_voltage_mv = struct.unpack("I", msg.data[0:4])[0]
        battery_status.regulated_current_ma = struct.unpack("I", msg.data[4:8])[0]
        self.regulated_publisher.publish(battery_status)
        self.get_logger().debug(
            f"Publishing {self.regulated_topic}: regulated_voltage_mv={battery_status.regulated_voltage_mv}mV, regulated_current_ma={battery_status.regulated_current_ma}mA"
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
