cmake_minimum_required(VERSION 3.5)
project(roborts_costmap)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

if(NOT DEFINED BUILD_PROTO)
  set(BUILD_PROTO ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(roborts_common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  CostmapProtoSrcs
  CostmapProtoHeaders
  ${ProtoFiles}
)

set(SRC_LIST
  ${CMAKE_CURRENT_SOURCE_DIR}/src/obstacle_layer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/layered_costmap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/costmap_math.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/costmap_interface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/observation_buffer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/layer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/inflation_layer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/costmap_2d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/static_layer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/footprint.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/costmap_layer.cpp
)

add_library(roborts_costmap SHARED
  ${SRC_LIST}
  ${CostmapProtoHeaders}
  ${CostmapProtoSrcs}
)

target_include_directories(roborts_costmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/proto>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(roborts_costmap
  rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs
  sensor_msgs nav_msgs geometry_msgs pcl_conversions
  laser_geometry roborts_common ament_index_cpp
)

target_link_libraries(roborts_costmap ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_costmap src/test_costmap.cpp)
ament_target_dependencies(test_costmap rclcpp tf2_ros roborts_common ament_index_cpp)
target_link_libraries(test_costmap roborts_costmap ${PROTOBUF_LIBRARIES})

install(TARGETS roborts_costmap EXPORT roborts_costmapTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS test_costmap DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs
  sensor_msgs nav_msgs geometry_msgs pcl_conversions
  laser_geometry roborts_common ament_index_cpp
)
ament_export_targets(roborts_costmapTargets HAS_LIBRARY_TARGET)
ament_export_libraries(roborts_costmap)

ament_package()
