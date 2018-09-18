cmake_minimum_required(VERSION 3.9)
project(rrt_star)

set(CMAKE_CXX_STANDARD 11)

#set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_executable(rrt_star
        test/main.cpp
        src/rrt_star.cpp
        )
target_link_libraries(rrt_star
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES}
        )