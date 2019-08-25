cmake_minimum_required(VERSION 3.13)
project(dijkstra)

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(dijkstra main.cpp dijkstra.hpp)
target_link_libraries(dijkstra
        ${OpenCV_LIBRARIES}
)
