cmake_minimum_required(VERSION 3.5)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ConstraintSetProtoSrc
  ConstraintSetProtoHds
  ${ProtoFiles})

add_library(constraint_set SHARED
  ${ConstraintSetProtoSrc}
  ${ConstraintSetProtoHds}
  constraint_set.cpp)

add_library(detection::constraint_set ALIAS constraint_set)

target_include_directories(constraint_set PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../util)

ament_target_dependencies(constraint_set
  rclcpp roborts_common ament_index_cpp cv_bridge image_transport sensor_msgs)

target_link_libraries(constraint_set
  detection::tool
  ${PROTOBUF_LIBRARIES}
  ${OpenCV_LIBS})

install(TARGETS constraint_set DESTINATION lib/${PROJECT_NAME})
