project(global_planner)

add_subdirectory(a_star_planner)
set(CMAKE_BUILD_TYPE Release)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto GlobalPlannerProtoSrc GlobalPlannerProtoHds ${ProtoFiles})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(global_planner_node
  ${GlobalPlannerProtoSrc}
  ${GlobalPlannerProtoHds}
  global_planner_node.cpp
)
target_include_directories(global_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(global_planner_node
  rclcpp rclcpp_action tf2 tf2_ros tf2_geometry_msgs
  nav_msgs geometry_msgs roborts_common roborts_msgs roborts_costmap ament_index_cpp
)

target_link_libraries(global_planner_node
  planning::global_planner::a_star_planner
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

add_executable(global_planner_test global_planner_test.cpp)
target_include_directories(global_planner_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(global_planner_test
  rclcpp rclcpp_action roborts_msgs roborts_common nav_msgs geometry_msgs
)

install(TARGETS global_planner_node global_planner_test
  DESTINATION lib/roborts_planning)

install(DIRECTORY config/ DESTINATION share/roborts_planning/global_planner/config)
