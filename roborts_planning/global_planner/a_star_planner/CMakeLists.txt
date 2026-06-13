project(a_star_planner)
set(CMAKE_BUILD_TYPE Release)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto AStarPlannerConfigProtoSrc AStarPlannerConfigProtoHds ${ProtoFiles})

add_library(${PROJECT_NAME}
  SHARED
  ${AStarPlannerConfigProtoSrc}
  ${AStarPlannerConfigProtoHds}
  a_star_planner.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp roborts_common roborts_costmap ament_index_cpp
  geometry_msgs nav_msgs
)
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
