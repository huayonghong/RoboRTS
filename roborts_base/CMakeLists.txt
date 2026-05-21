cmake_minimum_required(VERSION 3.5)
project(roborts_base)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roborts_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Glog REQUIRED)

add_executable(roborts_base_node
  roborts_base_node.cpp
  chassis/chassis.cpp
  gimbal/gimbal.cpp
  referee_system/referee_system.cpp
  roborts_sdk/dispatch/execution.cpp
  roborts_sdk/dispatch/handle.cpp
  roborts_sdk/protocol/protocol.cpp
  roborts_sdk/hardware/serial_device.cpp
)

ament_target_dependencies(roborts_base_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  nav_msgs geometry_msgs roborts_msgs
)

target_link_libraries(roborts_base_node
  Threads::Threads
  ${GLOG_LIBRARY}
)

install(TARGETS roborts_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
