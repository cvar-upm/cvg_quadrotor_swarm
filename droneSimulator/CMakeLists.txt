cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)
#add_definitions(-std=c++0x)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(DRONESIMULATOR_SOURCE_DIR
        src/source)

set(DRONESIMULATOR_INCLUDE_DIR
        src/include)

set(DRONESIMULATOR_HEADER_FILES
        ${DRONESIMULATOR_INCLUDE_DIR}/droneElements.h
        ${DRONESIMULATOR_INCLUDE_DIR}/droneSimulator.h
        ${DRONESIMULATOR_INCLUDE_DIR}/quadrotor_model.h
)

set(DRONESIMULATOR_SOURCE_FILES
        ${DRONESIMULATOR_SOURCE_DIR}/droneElements.cpp
        ${DRONESIMULATOR_SOURCE_DIR}/droneSimulator.cpp
        ${DRONESIMULATOR_SOURCE_DIR}/quadrotor_model.cpp
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set(OTHER_FILES
        manifest.xml
#        ${DRONESIMULATOR_INCLUDE_DIR}/config_Mydrone.h
)
add_custom_target(other_files SOURCES ${OTHER_FILES})

# droneSimulator library compilation
include_directories(${DRONESIMULATOR_INCLUDE_DIR})
rosbuild_add_library(droneSimulator ${DRONESIMULATOR_SOURCE_FILES} ${DRONESIMULATOR_HEADER_FILES})
