cmake_minimum_required(VERSION 3.5)

add_subdirectory(constraint_set)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ArmorDetectionProtoSrc
  ArmorDetectionProtoHds
  ${ProtoFiles})

add_executable(armor_detection_node
  ${ArmorDetectionProtoSrc}
  ${ArmorDetectionProtoHds}
  armor_detection_node.cpp
  gimbal_control.cpp)

target_include_directories(armor_detection_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../util)

target_link_libraries(armor_detection_node
  PRIVATE
  detection::tool
  detection::constraint_set
  ${PROTOBUF_LIBRARIES})

ament_target_dependencies(armor_detection_node
  rclcpp rclcpp_action cv_bridge image_transport sensor_msgs roborts_msgs
  roborts_common ament_index_cpp)

find_package(OpenCV REQUIRED)
target_link_libraries(armor_detection_node PRIVATE ${OpenCV_LIBS})

add_executable(armor_detection_client
  armor_detection_client.cpp)

ament_target_dependencies(armor_detection_client
  rclcpp rclcpp_action roborts_msgs)

install(TARGETS armor_detection_node armor_detection_client
  DESTINATION lib/${PROJECT_NAME})
