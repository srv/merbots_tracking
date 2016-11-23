cmake_minimum_required(VERSION 2.8.3)

project(merbots_tracking)

# Finding headers and launch files for QtCreator
file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR} *.cxx *.cpp *.c *.py)
file(GLOB_RECURSE LNCH ${CMAKE_CURRENT_SOURCE_DIR} *.launch)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge camera_calibration_parsers)

# System dependencies
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Defining the package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES struck kcf
    CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge camera_calibration_parsers
	DEPENDS OpenCV
)

###########
## Build ##
###########

# Including directories.
include_directories(include trackers ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Custom target to include headers and launch files in QtCreator
add_custom_target(dtarget_merbots_tracking SOURCES ${HDRS} ${SRCS} ${LNCH})

# STRUCK Tracker Library
add_subdirectory(trackers/struck)

# KCF Tracker Library
add_subdirectory(trackers/kcf)

# Sequence Publisher Node
add_executable(seq_publisher
        src/nodes/seq_publisher.cpp)
target_link_libraries(seq_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

# Target Selector
add_executable(target_selector
        src/nodes/target_selector.cpp)
target_link_libraries(target_selector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Detection and Tracking
add_executable(det_and_track
    src/util/Params.cpp
    src/util/PHOG.cpp
    src/util/SharedData.cpp
    src/threads/TargetDetector.cpp
    src/threads/TargetTracker.cpp
    include/merbots_tracking/wrappers/VisualTracker.h
    src/wrappers/StruckVisualTracker.cpp
    src/wrappers/KCFVisualTracker.cpp
    src/nodes/det_and_track.cpp
)
target_link_libraries(det_and_track ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} struck kcf)
