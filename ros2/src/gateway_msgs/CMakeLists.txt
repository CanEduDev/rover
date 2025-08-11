cmake_minimum_required(VERSION 3.8)
project(gateway_msgs)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelStatus.msg"
  "msg/ObstacleDistance.msg"
  "msg/ReportFrequency.msg"
  "msg/CANStatus.msg"
  "msg/BatteryStatus.msg"
)

ament_package()
