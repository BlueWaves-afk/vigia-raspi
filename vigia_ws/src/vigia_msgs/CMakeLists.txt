cmake_minimum_required(VERSION 3.16)
project(vigia_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/SpatialLatent.msg"
  "msg/DepthMap.msg"
  "msg/ImuSample.msg"
  "msg/GpsPvt.msg"
  "msg/SignedEt.msg"
  "msg/SensorHealth.msg"
  "msg/HazardEvent.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
