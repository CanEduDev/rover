import threading

import can
import rclpy
import rover
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool

# TODO: add timer to publish can_enabled regularly for boot up and node restart purposes.
# Nodes should default to disabled until this node is started.


class MayorNode(Node):
    def __init__(self):
        super().__init__("can_toggle_node")
        self.can_enabled = True
        self.can_enabled_lock = threading.Lock()

        # Publisher for CAN enabled state
        self.can_enabled_pub = self.create_publisher(
            Bool, "can_enabled", ReliabilityPolicy.RELIABLE
        )
        self.publish_can_enabled()

        self.can_bus = can.ThreadSafeBus(
            interface="socketcan", channel="can0", bitrate=125_000
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
        msg = Bool()
        msg.data = self.can_enabled
        self.can_enabled_pub.publish(msg)
        self.get_logger().debug(f"Published can_enabled: {msg.data}")

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
    rclpy.init(args=args)
    node = MayorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
