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
rosbuild_gensrv()

set(CVGLOGGERROSLIB_SOURCE_DIR
        src/source)

set(CVGLOGGERROSLIB_INCLUDE_DIR
        src/include)

set(CVGLOGGERROSLIB_HEADER_FILES
        ${CVGLOGGERROSLIB_INCLUDE_DIR}/droneloggerrospublisher.h
        ${CVGLOGGERROSLIB_INCLUDE_DIR}/droneloggerrostopicsubscriber.h
        ${CVGLOGGERROSLIB_INCLUDE_DIR}/droneloggerrosbasicmodule.h
)

set(CVGLOGGERROSLIB_SOURCE_FILES
        ${CVGLOGGERROSLIB_SOURCE_DIR}/droneloggerrospublisher.cpp
        ${CVGLOGGERROSLIB_SOURCE_DIR}/droneloggerrostopicsubscriber.cpp
        ${CVGLOGGERROSLIB_SOURCE_DIR}/droneloggerrosbasicmodule.cpp
)

include_directories(${CVGLOGGERROSLIB_INCLUDE_DIR})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(lib_cvgloggerROS ${CVGLOGGERROSLIB_SOURCE_FILES} ${CVGLOGGERROSLIB_HEADER_FILES})

add_executable(code_tests ${CVGLOGGERROSLIB_SOURCE_DIR}/code_tests.cpp)
target_link_libraries(code_tests lib_cvgloggerROS)

add_executable(code_tests2 ${CVGLOGGERROSLIB_SOURCE_DIR}/code_tests2.cpp)
target_link_libraries(code_tests2 lib_cvgloggerROS)

add_executable(code_tests3 ${CVGLOGGERROSLIB_SOURCE_DIR}/code_tests3.cpp)
target_link_libraries(code_tests3 lib_cvgloggerROS)

add_executable(code_tests4 ${CVGLOGGERROSLIB_SOURCE_DIR}/code_tests4.cpp)
target_link_libraries(code_tests4 lib_cvgloggerROS)

add_executable(code_tests5 ${CVGLOGGERROSLIB_SOURCE_DIR}/code_tests5.cpp)
target_link_libraries(code_tests5 lib_cvgloggerROS)

set(OTHER_FILES
        manifest.xml
        srv/logThisString.srv
)
add_custom_target(other_files SOURCES ${OTHER_FILES})
