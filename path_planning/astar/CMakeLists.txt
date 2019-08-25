cmake_minimum_required(VERSION 3.13)
project(astar)

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(astar main.cpp astar.hpp)
target_link_libraries(astar
        ${OpenCV_LIBRARIES}
)
