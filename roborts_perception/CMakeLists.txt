cmake_minimum_required(VERSION 3.8)
project(roborts_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(roborts_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(CUDAToolkit REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0)

# Jetson TensorRT has no CMake config; locate the library and headers.
find_library(TENSORRT_LIBRARY nvinfer REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include/aarch64-linux-gnu /usr/include
  REQUIRED)

# ---------------------------------------------------------------- perception
add_executable(person_gesture_node
  src/csi_camera.cpp
  src/camera_model.cpp
  src/trt_engine.cpp
  src/pose_detector.cpp
  src/person_tracker.cpp
  src/gesture_classifier.cpp
  src/person_gesture_node.cpp
)
target_include_directories(person_gesture_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${TENSORRT_INCLUDE_DIR}
)
target_link_libraries(person_gesture_node
  PkgConfig::GSTREAMER
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  ${TENSORRT_LIBRARY}
  CUDA::cudart
)
ament_target_dependencies(person_gesture_node
  rclcpp
  sensor_msgs
  builtin_interfaces
  ament_index_cpp
  geometry_msgs
  std_msgs
  roborts_msgs
)

# ------------------------------------------------------------ engine testing
add_executable(pose_smoke_test
  src/trt_engine.cpp
  src/pose_detector.cpp
  src/pose_smoke_test.cpp
)
target_include_directories(pose_smoke_test PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
)
target_link_libraries(pose_smoke_test
  ${OpenCV_LIBS}
  ${TENSORRT_LIBRARY}
  CUDA::cudart
)
ament_target_dependencies(pose_smoke_test rclcpp)

# ------------------------------------------------------------------- gimbal
add_executable(gimbal_follow_node src/gimbal_follow_node.cpp)
target_include_directories(gimbal_follow_node PRIVATE include)
ament_target_dependencies(gimbal_follow_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  roborts_msgs
)

# ------------------------------------------------------------ gesture bridge
add_executable(gesture_command_bridge
  src/gesture_command_bridge.cpp
  src/gesture_classifier.cpp
)
target_include_directories(gesture_command_bridge PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(gesture_command_bridge ${OpenCV_LIBS})
ament_target_dependencies(gesture_command_bridge
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  roborts_msgs
)

# --------------------------------------------------------------- cmd_vel mux
add_executable(cmd_vel_mux src/cmd_vel_mux.cpp)
ament_target_dependencies(cmd_vel_mux
  rclcpp
  geometry_msgs
  roborts_msgs
)

install(TARGETS
  person_gesture_node
  pose_smoke_test
  gimbal_follow_node
  gesture_command_bridge
  cmd_vel_mux
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

# Offline tools.  csi_camera.py is imported by calibrate_csi.py from the same
# directory, so both land in lib/<package>.
install(PROGRAMS
  scripts/calibrate_csi.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibrate_csi
)
install(FILES scripts/csi_camera.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/build_models.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME build_models
)
install(PROGRAMS
  scripts/perception_view.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_view
)

ament_package()
