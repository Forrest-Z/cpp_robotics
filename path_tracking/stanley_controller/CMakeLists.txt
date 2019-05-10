cmake_minimum_required(VERSION 3.12)
project(stanley_controller)

set(CMAKE_CXX_STANDARD 14)

find_package(PythonLibs 2.7)

include_directories(include lib ${PYTHON_INCLUDE_DIRS})

add_executable(stanley_controller test/main.cpp)
target_link_libraries(stanley_controller ${PYTHON_LIBRARIES})