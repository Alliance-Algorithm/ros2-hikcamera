cmake_minimum_required(VERSION 3.22)
project(hikcamera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

find_package(OpenCV 4.5 REQUIRED)
link_directories(src/sdk/lib)

set(PROJECT_SOURCES
    src/capturer.cpp
    src/image_capturer.cpp
)
ament_auto_add_library (
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCES}
)

include_directories(
    include
    src
)

install(
    DIRECTORY src/sdk/lib/
    DESTINATION lib
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl)

ament_auto_package()
