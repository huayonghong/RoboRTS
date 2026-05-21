project(a_star_planner)
set(CMAKE_BUILD_TYPE Release)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto AStarPlannerConfigProtoSrc AStarPlannerConfigProtoHds ${ProtoFiles})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_library(${PROJECT_NAME} SHARED
  ${AStarPlannerConfigProtoSrc}
  ${AStarPlannerConfigProtoHds}
  a_star_planner.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC roborts_costmap ${PROTOBUF_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp roborts_common ament_index_cpp
)
add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
