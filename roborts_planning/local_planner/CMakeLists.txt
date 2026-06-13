cmake_minimum_required(VERSION 3.0)
project(local_planner)


set(EXTERNAL_INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  )
find_package(Boost REQUIRED COMPONENTS thread)
set(EXTERNAL_LIBS
  ${SUITESPARSE_LIBRARIES}
  g2o::core
  g2o::stuff
  g2o::types_slam2d
  g2o::types_slam3d
  g2o::solver_csparse
  g2o::solver_cholmod
  g2o::csparse_extension
  ${PROTOBUF_LIBRARIES}
  Boost::thread
  )

include_directories(${EXTERNAL_INCLUDE_DIRS} include/ timed_elastic_band/include/)

add_subdirectory(timed_elastic_band)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto
  LocalPlanProtoSrc
  LocalPlanProtoHds
  ${ProtoFiles}
  )

aux_source_directory(src/.  SRC_LIST)

#local planner node
add_executable(${PROJECT_NAME}_node
  ${LocalPlanProtoSrc}
  ${LocalPlanProtoHds}
  src/local_planner_node.cpp
  src/local_visualization.cpp
  src/obstacles.cpp
  src/odom_info.cpp
  src/robot_position_cost.cpp
  )

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp rclcpp_action tf2 tf2_ros tf2_geometry_msgs
  nav_msgs geometry_msgs visualization_msgs
  roborts_common roborts_msgs roborts_costmap ament_index_cpp
)
target_link_libraries(${PROJECT_NAME}_node
  planning::timed_elastic_band ${EXTERNAL_LIBS})

#local palnner test
add_executable(teb_test
  ${LocalPlanProtoSrc}
  ${LocalPlanProtoHds}
  src/teb_test.cpp
  src/local_visualization.cpp
  src/obstacles.cpp
  src/odom_info.cpp
  src/robot_position_cost.cpp
  )

ament_target_dependencies(teb_test
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  nav_msgs geometry_msgs visualization_msgs interactive_markers
  roborts_common roborts_msgs roborts_costmap ament_index_cpp
)
target_link_libraries(teb_test
  planning::timed_elastic_band ${EXTERNAL_LIBS})

#vel converter
add_executable(vel_converter
  src/vel_converter.cpp
  )

ament_target_dependencies(vel_converter
  rclcpp geometry_msgs roborts_msgs
)

install(TARGETS ${PROJECT_NAME}_node teb_test vel_converter
  DESTINATION lib/roborts_planning)
install(DIRECTORY config/ DESTINATION share/roborts_planning/local_planner/config)
