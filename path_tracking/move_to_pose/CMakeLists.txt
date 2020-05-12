cmake_minimum_required(VERSION 3.12)
project(move_to_pose)

set(CMAKE_CXX_STANDARD 14)

find_package(PythonLibs 2.7)

include_directories(${PYTHON_INCLUDE_DIRS})

add_executable(move_to_pose main.cc)
target_link_libraries(move_to_pose ${PYTHON_LIBRARIES})
