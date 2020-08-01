cmake_minimum_required(VERSION 3.16)
project(path_smooth)

set(CMAKE_CXX_STANDARD 11)

find_package(PythonLibs 2.7)

include_directories(
        ${PYTHON_INCLUDE_DIRS}
)

add_executable(path_smooth main.cc)
target_link_libraries(path_smooth ${PYTHON_LIBRARIES})