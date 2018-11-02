cmake_minimum_required(VERSION 3.9)
project(closed_loop_rrt_star)

set(CMAKE_CXX_STANDARD 11)

if (EXISTS "FindOpenCV.cmake")
    message("find systerm opencv fail, use local.")
else (EXISTS "FindOpenCV.cmake")
    set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)
endif (EXISTS "FindOpenCV.cmake")

find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_executable(closed_loop_rrt_star
        test/main.cpp
        src/closed_loop_rrt_star.cpp
        src/reeds_shepp_path_planning.cpp
        )
target_link_libraries(closed_loop_rrt_star
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES}
        )