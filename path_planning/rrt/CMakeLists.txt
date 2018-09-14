cmake_minimum_required(VERSION 3.9)
project(rrt)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple_rrt simple_rrt.cpp)
target_link_libraries(simple_rrt
        ${OpenCV_LIBRARIES}
)

add_executable(rrt_with_pathsmoothing rrt_with_pathsmoothing.cpp)
target_link_libraries(rrt_with_pathsmoothing
        ${OpenCV_LIBRARIES}
)