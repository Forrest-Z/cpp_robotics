cmake_minimum_required(VERSION 3.9)
project(ukf_slam)

set(CMAKE_CXX_STANDARD 11)

find_package(PythonLibs 2.7)

set(source  src/main.cpp
        src/ukf_slam.cpp
            src/tools.cpp)

add_executable(ukf_slam ${source})
target_include_directories(ukf_slam PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(ukf_slam ${PYTHON_LIBRARIES})