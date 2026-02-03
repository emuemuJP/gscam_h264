cmake_minimum_required(VERSION 3.6)

project(gscam_h264)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# GStreamer doesn't provide CMake files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 IMPORTED_TARGET)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0 IMPORTED_TARGET)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

#=============
# H264 Camera Publisher executable
#=============

add_executable(h264_camera_publisher_main
  src/h264_camera_publisher.cpp
  src/h264_camera_publisher_main.cpp
)

ament_target_dependencies(h264_camera_publisher_main
  rclcpp
  sensor_msgs
)

target_link_libraries(h264_camera_publisher_main
  PkgConfig::GSTREAMER
  PkgConfig::GST_APP
)

target_include_directories(h264_camera_publisher_main PUBLIC
  "$<BUILD_INTERFACE:${GSTREAMER_INCLUDE_DIRS}>")

#=============
# Install
#=============

install(
  TARGETS h264_camera_publisher_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch cfg DESTINATION share/${PROJECT_NAME})

ament_package()
