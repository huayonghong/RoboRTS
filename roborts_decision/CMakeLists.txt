cmake_minimum_required(VERSION 3.5)
project(roborts_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

if(NOT DEFINED BUILD_PROTO)
  set(BUILD_PROTO ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(roborts_common REQUIRED)
find_package(roborts_msgs REQUIRED)
find_package(roborts_costmap REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc DecisionProtoHds ${ProtoFiles})

add_executable(behavior_test_node
  ${DecisionProtoHds} ${DecisionProtoSrc}
  behavior_test.cpp
  blackboard/blackboard.cpp
  executor/chassis_executor.cpp
)

target_include_directories(behavior_test_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(behavior_test_node PRIVATE
  ${PROTOBUF_LIBRARIES}
  Eigen3::Eigen)

ament_target_dependencies(behavior_test_node
  rclcpp rclcpp_action tf2 tf2_ros tf2_geometry_msgs
  nav_msgs geometry_msgs action_msgs
  roborts_common roborts_msgs roborts_costmap ament_index_cpp)

install(TARGETS behavior_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
