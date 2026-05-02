cmake_minimum_required(VERSION 3.22)
project(hikcamera VERSION 2.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

find_package(OpenCV 4.5 REQUIRED)

if(CMAKE_SYSROOT)
  set(HIK_MVS_SDK_ROOT "${CMAKE_SYSROOT}/opt/mvs-usb3-core")
else()
  set(HIK_MVS_SDK_ROOT "/opt/mvs-usb3-core")
endif()

find_package(MVSUSB3Core REQUIRED CONFIG
  PATHS "${HIK_MVS_SDK_ROOT}" NO_DEFAULT_PATH
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/capturer.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  ${OpenCV_LIBS}
  MVSUSB3Core::MVSUSB3Core
)

ament_auto_package()
