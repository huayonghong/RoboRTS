cmake_minimum_required(VERSION 3.5)
project(roborts_localization)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

add_subdirectory(amcl)

add_executable(localization_node
  localization_node.cpp
  localization_node.h
  localization_math.cpp
)

target_include_directories(localization_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(localization_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  nav_msgs sensor_msgs geometry_msgs
  message_filters ament_index_cpp
)
target_link_libraries(localization_node
  localization::amcl
  ${EIGEN3_LIBRARIES}
  ${GLOG_LIBRARY}
)

install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY amcl/config/ DESTINATION share/${PROJECT_NAME}/amcl/config)

ament_package()
