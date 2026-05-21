cmake_minimum_required(VERSION 3.5)
project(roborts_tracking)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roborts_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_library(kcf_tracker STATIC KCFcpp/src/fhog.cpp KCFcpp/src/kcftracker.cpp)

target_include_directories(kcf_tracker PRIVATE ${OpenCV_INCLUDE_DIRS} KCFcpp/src)
target_link_libraries(kcf_tracker ${OpenCV_LIBRARIES})

add_executable(roborts_tracking_test tracking_test.cpp tracking_utility.cpp)
target_include_directories(roborts_tracking_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(roborts_tracking_test kcf_tracker ${OpenCV_LIBRARIES})
ament_target_dependencies(roborts_tracking_test rclcpp roborts_msgs)

install(TARGETS roborts_tracking_test DESTINATION lib/${PROJECT_NAME})

ament_package()
