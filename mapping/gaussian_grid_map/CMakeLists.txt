cmake_minimum_required(VERSION 3.9)
project(gaussian_grid_map)

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

include_directories(
        include
        lib
        ${OpenCV_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        )

add_executable(gaussian_grid_map
        test/main.cpp
        src/gaussian_grid_map.cpp
        )
target_link_libraries(gaussian_grid_map
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES}
        )