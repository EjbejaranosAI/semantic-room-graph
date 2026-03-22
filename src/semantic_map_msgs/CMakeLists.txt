cmake_minimum_required(VERSION 3.8)
project(semantic_map_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RegionNode.msg"
  "msg/RegionEdge.msg"
  "msg/SemanticGraph.msg"
  "msg/RegionObservation.msg"
  "msg/OCRResult.msg"
  "msg/VLMResult.msg"
  "msg/QueryRequest.msg"
  "msg/QueryCandidate.msg"
  "msg/QueryResult.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
