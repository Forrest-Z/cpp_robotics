cmake_minimum_required(VERSION 3.9)
project(DubinsPath)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(DubinsPath dubins_path_planning.cpp)
target_link_libraries(DubinsPath
        ${OpenCV_LIBRARIES}
)