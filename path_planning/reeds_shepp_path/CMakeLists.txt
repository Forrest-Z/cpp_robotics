cmake_minimum_required(VERSION 3.9)
project(reeds_shepp_path)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_executable(reeds_shepp_path
        test/main.cpp
        src/reeds_shepp_path_planning.cpp
)
target_link_libraries(reeds_shepp_path
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES}
)