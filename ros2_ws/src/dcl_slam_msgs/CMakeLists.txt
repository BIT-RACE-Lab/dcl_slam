cmake_minimum_required(VERSION 3.5)
project(dcl_slam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GlobalDescriptor.msg"
  "msg/LoopInfo.msg"
  "msg/NeighborEstimate.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
