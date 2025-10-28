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
link_directories(src/sdk/lib)

include_directories(include src)
ament_auto_add_library (
    ${PROJECT_NAME} SHARED
    src/capturer.cpp
)
target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    MvCameraControl
)
install(
    DIRECTORY src/sdk/lib/
    DESTINATION lib
)
ament_auto_package()
