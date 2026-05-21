cmake_minimum_required(VERSION 3.5)
project(roborts_common)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roborts_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_export_dependencies(rclcpp ament_index_cpp)

ament_package()
