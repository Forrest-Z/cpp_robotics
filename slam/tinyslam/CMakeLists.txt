cmake_minimum_required(VERSION 2.8.3)
project(tinyslam)

## C++11
add_compile_options(-std=c++11)

## set HEADERS and SOURCES
set(SOURCES
        src/CoreSLAM_loop_closing.cpp
		src/CoreSLAM_state.cpp
		src/CoreSLAM_random.cpp
		src/CoreSLAM.cpp)
set(HEADERS
        include/CoreSLAM.h)
    
## OPENCV include and LIBRARY
find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

## executable
add_executable(tinyslam src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(tinyslam ${OpenCV_LIBRARIES})

