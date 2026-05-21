cmake_minimum_required(VERSION 3.5)
project(roborts_detection)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

if(NOT DEFINED BUILD_PROTO)
  set(BUILD_PROTO ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roborts_msgs REQUIRED)
find_package(roborts_common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} util)
add_subdirectory(util)
add_subdirectory(armor_detection)

install(DIRECTORY armor_detection/config
  DESTINATION share/${PROJECT_NAME}/armor_detection/config)
install(DIRECTORY armor_detection/constraint_set/config
  DESTINATION share/${PROJECT_NAME}/armor_detection/constraint_set/config)

ament_package()
