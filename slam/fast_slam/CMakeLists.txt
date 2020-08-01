cmake_minimum_required(VERSION 3.9)
project(fast_slam)

set(CMAKE_CXX_STANDARD 11)

find_package(PythonLibs 2.7)

include_directories(include lib)

set(source
        src/main.cpp
        src/fast_slam.cpp
        src/tools.cpp)

add_executable(fast_slam ${source})
target_include_directories(fast_slam PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(fast_slam ${PYTHON_LIBRARIES})



