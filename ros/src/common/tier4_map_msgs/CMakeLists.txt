cmake_minimum_required(VERSION 3.16)
project(tier4_map_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapProjectorInfo.msg"
  DEPENDENCIES
    geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
