project(detection_util)

add_library(cv_toolbox INTERFACE)

target_include_directories(cv_toolbox
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_library(detection::tool ALIAS cv_toolbox)
