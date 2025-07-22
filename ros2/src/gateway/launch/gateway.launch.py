from launch import LaunchDescription  # pyright: ignore[reportAttributeAccessIssue]
from launch.actions import (  # pyright: ignore[reportMissingImports]
    DeclareLaunchArgument,
)
from launch.substitutions import (  # pyright: ignore[reportMissingImports]
    LaunchConfiguration,
)
from launch_ros.actions import Node  # pyright: ignore[reportMissingImports]


def generate_launch_description():
    package = "gateway"
    namespace = "rover"

    logger_arg = DeclareLaunchArgument("log_level", default_value="info")

    default_logger_arg = ["--ros-args", "--log-level", LaunchConfiguration("log_level")]

    return LaunchDescription(
        [
            logger_arg,
            # Mayor node
            Node(
                package=package,
                namespace=namespace,
                executable="mayor_node",
                name="mayor",
                arguments=default_logger_arg,
            ),
            # Radio node
            Node(
                package=package,
                namespace=namespace,
                executable="radio_node",
                name="radio",
                arguments=default_logger_arg,
            ),
            # Controller node
            Node(
                package=package,
                namespace=namespace,
                executable="controller_node",
                name="controller",
                arguments=default_logger_arg,
            ),
            # Battery Monitor nodes
            Node(
                package=package,
                namespace=namespace,
                executable="battery_node",
                name="battery_monitor_control_system",
                parameters=[{"position": "control_system"}],
                arguments=default_logger_arg,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="battery_node",
                name="battery_monitor_ad_system",
                parameters=[{"position": "ad_system"}],
                arguments=default_logger_arg,
            ),
            # Wheel nodes
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_front_left",
                parameters=[{"position": "front_left"}],
                arguments=default_logger_arg,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_front_right",
                parameters=[{"position": "front_right"}],
                arguments=default_logger_arg,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_rear_left",
                parameters=[{"position": "rear_left"}],
                arguments=default_logger_arg,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_rear_right",
                parameters=[{"position": "rear_right"}],
                arguments=default_logger_arg,
            ),
            # Obstacle Detector nodes
            Node(
                package=package,
                namespace=namespace,
                executable="obstacle_detector_node",
                name="obstacle_detector_front",
                parameters=[{"position": "front"}],
                arguments=default_logger_arg,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="obstacle_detector_node",
                name="obstacle_detector_rear",
                parameters=[{"position": "rear"}],
                arguments=default_logger_arg,
            ),
        ]
    )
