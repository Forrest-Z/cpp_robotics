cmake_minimum_required(VERSION 3.9)
project(ekf_slam)

set(CMAKE_CXX_STANDARD 11)

find_package(PythonLibs 2.7)

include_directories(include lib)

set(source
        src/main.cpp
        src/ekf_slam.cpp
        src/tools.cpp)

add_executable(ekf_slam ${source})
target_include_directories(ekf_slam PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(ekf_slam ${PYTHON_LIBRARIES})

