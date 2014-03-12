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

set(DRONEEKFSTATEESTIMATORROSMODULE_SOURCE_DIR
        src/source)

set(DRONEEKFSTATEESTIMATORROSMODULE_INCLUDE_DIR
        src/include)

set(DRONEEKFSTATEESTIMATORROSMODULE_HEADER_FILES
        ${DRONEEKFSTATEESTIMATORROSMODULE_INCLUDE_DIR}/droneEKFStateEstimatorROSModule.h
)

set(DRONEEKFSTATEESTIMATORROSMODULE_SOURCE_FILES
        ${DRONEEKFSTATEESTIMATORROSMODULE_SOURCE_DIR}/droneEKFStateEstimatorROSModule.cpp
        ${DRONEEKFSTATEESTIMATORROSMODULE_SOURCE_DIR}/droneEKFStateEstimatorROSModuleNode.cpp
        #${DRONEEKFSTATEESTIMATORROSMODULE_SOURCE_DIR}/droneEKFStateEstimatorROSModuleNodeWLog.cpp
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
#        srv/setInitDroneYaw_srv_type.srv   // It is now located in droneMsgsROS
)
add_custom_target(other_files SOURCES ${OTHER_FILES})

include_directories(${DRONEEKFSTATEESTIMATORROSMODULE_INCLUDE_DIR})
rosbuild_add_executable(droneEKFStateEstimatorROSModuleNode ${DRONEEKFSTATEESTIMATORROSMODULE_SOURCE_FILES} ${DRONEEKFSTATEESTIMATORROSMODULE_HEADER_FILES})
