cmake_minimum_required(VERSION 3.5)
project(timed_elastic_band)

aux_source_directory(src/. SRC_LIST)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto/*.proto")
rrts_protobuf_generate_cpp(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto
  timed_elastic_bandProtoSrc timed_elastic_bandProtoHds ${ProtoFiles})

add_library(${PROJECT_NAME} SHARED
  ${timed_elastic_bandProtoSrc} ${timed_elastic_bandProtoHds} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} PUBLIC ${EXTERNAL_LIBS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  nav_msgs geometry_msgs visualization_msgs
  roborts_common roborts_msgs roborts_costmap ament_index_cpp
)
add_library(planning::timed_elastic_band ALIAS timed_elastic_band)
