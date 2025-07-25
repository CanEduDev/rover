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

    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="socketcan"
    )
    can_channel_arg = DeclareLaunchArgument("can_channel", default_value="can0")
    can_bitrate_arg = DeclareLaunchArgument("can_bitrate", default_value="125000")

    log_level_arg = DeclareLaunchArgument("log_level", default_value="info")

    default_args = [
        # Node args
        "--interface",
        LaunchConfiguration("can_interface"),
        "--channel",
        LaunchConfiguration("can_channel"),
        "--bitrate",
        LaunchConfiguration("can_bitrate"),
        # ROS args
        "--ros-args",
        "--log-level",
        LaunchConfiguration("log_level"),
    ]

    return LaunchDescription(
        [
            log_level_arg,
            can_interface_arg,
            can_channel_arg,
            can_bitrate_arg,
            # Mayor node
            Node(
                package=package,
                namespace=namespace,
                executable="mayor_node",
                name="mayor",
                arguments=default_args,
            ),
            # Radio node
            Node(
                package=package,
                namespace=namespace,
                executable="radio_node",
                name="radio",
                arguments=default_args,
            ),
            # Controller node
            Node(
                package=package,
                namespace=namespace,
                executable="controller_node",
                name="controller",
                arguments=default_args,
            ),
            # Battery Monitor nodes
            Node(
                package=package,
                namespace=namespace,
                executable="battery_node",
                name="battery_monitor_control_system",
                parameters=[{"position": "control_system"}],
                arguments=default_args,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="battery_node",
                name="battery_monitor_ad_system",
                parameters=[{"position": "ad_system"}],
                arguments=default_args,
            ),
            # Wheel nodes
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_front_left",
                parameters=[{"position": "front_left"}],
                arguments=default_args,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_front_right",
                parameters=[{"position": "front_right"}],
                arguments=default_args,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_rear_left",
                parameters=[{"position": "rear_left"}],
                arguments=default_args,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="wheel_node",
                name="wheel_rear_right",
                parameters=[{"position": "rear_right"}],
                arguments=default_args,
            ),
            # Obstacle Detector nodes
            Node(
                package=package,
                namespace=namespace,
                executable="obstacle_detector_node",
                name="obstacle_detector_front",
                parameters=[{"position": "front"}],
                arguments=default_args,
            ),
            Node(
                package=package,
                namespace=namespace,
                executable="obstacle_detector_node",
                name="obstacle_detector_rear",
                parameters=[{"position": "rear"}],
                arguments=default_args,
            ),
        ]
    )
