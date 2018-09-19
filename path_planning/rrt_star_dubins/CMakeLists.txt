cmake_minimum_required(VERSION 3.9)
project(rrt_star_dubins)

set(CMAKE_CXX_STANDARD 11)

#set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_executable(rrt_star_dubins
        src/rrt_star_dubins.cpp
        test/main.cpp
        )

target_link_libraries(rrt_star_dubins
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES}
        )
