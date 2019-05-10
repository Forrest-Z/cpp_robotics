cmake_minimum_required(VERSION 3.9)
project(extended_kalman_filter)

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

include_directories(
        include
        lib
#        ${OpenCV_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_executable(extended_kalman_filter
        src/extended_kalman_filter.cpp
        test/main.cpp
)
#target_include_directories(dynamic_window_approach PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(extended_kalman_filter
#        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES}
)