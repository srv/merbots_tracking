cmake_minimum_required(VERSION 2.8.3)

project(kcf)

if(NOT WIN32)
    ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

find_package(OpenCV REQUIRED)

include_directories(
    src
    ${OpenCV_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE sourcefiles "*.h" "*.hpp" "*.cpp")

add_library(kcf ${sourcefiles})
target_link_libraries(kcf ${OpenCV_LIBS})
