cmake_minimum_required(VERSION 3.13)
project(a_star)

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(a_star main.cpp a_star.hpp)
target_link_libraries(a_star
        ${OpenCV_LIBRARIES}
)
