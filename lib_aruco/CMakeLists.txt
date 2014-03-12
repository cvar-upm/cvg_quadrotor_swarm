cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(lib_aruco)

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
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv() 
 
#opencv with no ros dependency
#find_package(OpenCV REQUIRED)


#STANDARD ARUCO LIB
set(LIB_ARUCO_SOURCE_DIR
	src/source) 
	
set(LIB_ARUCO_INCLUDE_DIR
	src/include)

set(LIB_ARUCO_SOURCE_FILES
        ${LIB_ARUCO_SOURCE_DIR}/arucofidmarkers.cpp
        ${LIB_ARUCO_SOURCE_DIR}/cvdrawingutils.cpp
        ${LIB_ARUCO_SOURCE_DIR}/marker.cpp
        ${LIB_ARUCO_SOURCE_DIR}/markerdetector.cpp
        ${LIB_ARUCO_SOURCE_DIR}/cameraparameters.cpp
        ${LIB_ARUCO_SOURCE_DIR}/boarddetector.cpp
        ${LIB_ARUCO_SOURCE_DIR}/board.cpp
)
 
set(LIB_ARUCO_HEADER_FILES
        ${LIB_ARUCO_INCLUDE_DIR}/aruco.h
        ${LIB_ARUCO_INCLUDE_DIR}/arucofidmarkers.h
        ${LIB_ARUCO_INCLUDE_DIR}/cvdrawingutils.h
        ${LIB_ARUCO_INCLUDE_DIR}/marker.h
        ${LIB_ARUCO_INCLUDE_DIR}/markerdetector.h
        ${LIB_ARUCO_INCLUDE_DIR}/boarddetector.h
        ${LIB_ARUCO_INCLUDE_DIR}/cameraparameters.h
        ${LIB_ARUCO_INCLUDE_DIR}/board.h
        ${LIB_ARUCO_INCLUDE_DIR}/exports.h
)






include_directories(${LIB_ARUCO_INCLUDE_DIR})

 
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread) 

rosbuild_add_library(lib_aruco ${LIB_ARUCO_SOURCE_FILES} ${LIB_ARUCO_HEADER_FILES})

#with no ros dependency
#add_library(lib_aruco ${LIB_ARUCO_SOURCE_FILES} ${LIB_ARUCO_HEADER_FILES})



#rosbuild_add_executable(ArUcoBoarder src/aruco_test_board.cpp ${ARUCO_SOURCE_FILES} ${ARUCO_HEADER_FILES})
#rosbuild_add_executable(ArUcoMarker src/aruco_simple.cpp ${ARUCO_SOURCE_FILES} ${ARUCO_HEADER_FILES})
#target_link_libraries(aruco ${PROJECT_NAME})

#with no ros dependency
#target_link_libraries(lib_aruco ${OpenCV_LIBS})


#installation with no ros dependency
#install(
#    TARGETS ${PROJECT_NAME}
#    EXPORT ${PROJECT_NAME}
#    LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib
#    RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/bin
#)




set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})
