cmake_minimum_required(VERSION 3.5)
project(roborts_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roborts_common REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(ProtoBuf REQUIRED)
find_package(OpenCV REQUIRED)

protobuf_generate_cpp(CAMERA_PROTO_SRCS CAMERA_PROTO_HDRS
  proto/camera_param.proto)

add_executable(camera_node
  camera_node.cpp
  camera_param.cpp
  uvc/uvc_driver.cpp
  ${CAMERA_PROTO_SRCS}
)

target_include_directories(camera_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/uvc
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_node
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  roborts_common
  ament_index_cpp
)
target_link_libraries(camera_node ${PROTOBUF_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_capture_test
  test/image_capture.cpp
)

ament_target_dependencies(image_capture_test
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
)
target_include_directories(image_capture_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_capture_test ${OpenCV_LIBRARIES})

install(TARGETS camera_node image_capture_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY proto/ DESTINATION share/${PROJECT_NAME}/proto)

ament_package()
