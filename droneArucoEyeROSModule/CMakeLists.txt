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

#find_library(AUTOOPTS_LIB opts)

#ADD_CUSTOM_COMMAND(
#  OUTPUT optmarkerServer.h optmarkerServer.c
#  COMMAND autogen ${PROJECT_SOURCE_DIR}/src/source/optmarkerServer.def
#  DEPENDS ${PROJECT_SOURCE_DIR}/src/source/optmarkerServer.def
#)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv() 
 


set(PARROTARUCOEYE_SOURCE_DIR
	src/source) 
	
set(PARROTARUCOEYE_INCLUDE_DIR
	src/include)

set(PARROTARUCOEYE_SOURCE_FILES
	${PARROTARUCOEYE_SOURCE_DIR}/droneArucoEyeROSModule.cpp 
	${PARROTARUCOEYE_SOURCE_DIR}/droneArucoEyeROSModuleNode.cpp 
)
 
set(PARROTARUCOEYE_HEADER_FILES
	${PARROTARUCOEYE_INCLUDE_DIR}/droneArucoEyeROSModule.h
)

#set(MARKERSERVER_SOURCE_FILES
#	optmarkerServer.c
#	${PARROTARUCOEYE_SOURCE_DIR}/rosutil.cc
#	${PARROTARUCOEYE_SOURCE_DIR}/markerServer.cpp
#)

#set(MARKERSERVER_HEADER_FILES
#)


include_directories(${PARROTARUCOEYE_INCLUDE_DIR})

 
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread) 
rosbuild_add_executable(droneArucoEyeROSModule ${PARROTARUCOEYE_SOURCE_FILES} ${PARROTARUCOEYE_HEADER_FILES})
#rosbuild_add_executable(markerServer ${MARKERSERVER_SOURCE_FILES} ${MARKERSERVER_HEADER_FILES})
#target_link_libraries(markerServer ${AUTOOPTS_LIB})
#rosbuild_add_executable(ArUcoBoarder src/aruco_test_board.cpp ${ARUCO_SOURCE_FILES} ${ARUCO_HEADER_FILES})
#rosbuild_add_executable(ArUcoMarker src/aruco_simple.cpp ${ARUCO_SOURCE_FILES} ${ARUCO_HEADER_FILES})
#target_link_libraries(aruco ${PROJECT_NAME})

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})
