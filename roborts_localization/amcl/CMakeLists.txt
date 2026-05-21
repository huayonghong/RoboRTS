project(amcl)

add_library(amcl
  amcl.cpp
  amcl.h
  amcl_config.h
  ${CMAKE_CURRENT_SOURCE_DIR}/map/amcl_map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_gaussian_pdf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_kdtree.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_sample.h
  ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_laser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_odom.cpp)

target_include_directories(amcl PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(amcl
  rclcpp tf2 tf2_ros nav_msgs sensor_msgs geometry_msgs message_filters tf2_geometry_msgs
)
target_link_libraries(amcl
  ${EIGEN3_LIBRARIES}
  ${GLOG_LIBRARY}
)
add_library(localization::amcl ALIAS amcl)
