cmake_minimum_required(VERSION 3.9)
project(iterative_closest_point)

set(CMAKE_CXX_STANDARD 11)

find_package(PythonLibs 2.7)

include_directories(
        include
        lib
        ${PYTHON_INCLUDE_DIRS}
)

add_executable(iterative_closest_point
        main.cpp
)
target_link_libraries(iterative_closest_point
        ${PYTHON_LIBRARIES}
)