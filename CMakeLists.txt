cmake_minimum_required(VERSION 2.8.3)

project(merbots_tracking)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge)

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
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES struck 
    CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge
    DEPENDS
)

###########
## Build ##
###########

# Including directories.
include_directories(include trackers ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# STRUCK Tracker Library
add_subdirectory(
	trackers/struck)

# STRUCK Tracker Library
add_subdirectory(
        trackers/kcf)
