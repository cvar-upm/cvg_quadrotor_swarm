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

#find_package(DRONE_INTERFACE REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#http://answers.ros.org/question/11294/a-problem-of-linking-external-libraries/
#gcc -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__ -Wall -pipe -lnative -L/usr/xenomai/lib -lxenomai -lpthread -lrtdk hellworld.c -o helloworld
#link_directories(/usr/xenomai/lib)
#rosbuild_add_executable(listener src/listener.cpp)
#include_directories(/usr/xenomai/include)
#add_definitions(-D_GNU_SOURCE -D_REENTRANT -D__XENO__)
#rosbuild_add_compile_flags(listener -Wall -pipe)
#target_link_libraries(listener native xenomai pthread rtdk)



set(DRONE_INTERFACE_SOURCE_DIR
        src/sources
)
	
set(DRONE_INTERFACE_INCLUDE_DIR
	src/includes
)

set(DRONE_INTERFACE_SOURCE_FILES
        ${DRONE_INTERFACE_SOURCE_DIR}/droneInterfaceROSModule.cpp
        ${DRONE_INTERFACE_SOURCE_DIR}/droneInterfaceROSModuleNode.cpp
)
	
set(DRONE_INTERFACE_HEADER_FILES
        ${DRONE_INTERFACE_INCLUDE_DIR}/droneInterfaceROSModule.h
)

set(DRONE_INTERFACE_JP_SOURCE_FILES
        ${DRONE_INTERFACE_SOURCE_DIR}/droneInterfaceROSModule.cpp
        ${DRONE_INTERFACE_SOURCE_DIR}/droneInterface_jp_ROSModuleNode.cpp
)

set(DRONE_INTERFACE_JP_HEADER_FILES
        ${DRONE_INTERFACE_INCLUDE_DIR}/droneInterfaceROSModule.h
)

#external libraries
#ncurses
link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include)

include_directories(${DRONE_INTERFACE_INCLUDE_DIR})

# Boost library
rosbuild_add_boost_directories()

#rosbuild_add_executable(parrotInterface src/interface.cpp src/Drone.cpp src/Drone.h)
rosbuild_add_executable(droneInterfaceROSModule ${DRONE_INTERFACE_SOURCE_FILES} ${DRONE_INTERFACE_HEADER_FILES})
#rosbuild_add_executable(parrotInterface_im_dataset_acq src/sources/interface_im_dataset_acq.cpp)

#rosbuild_add_executable(parrotInterfaceCommand src/interfaceCommand.cpp)


#external libraries
#ncurses
target_link_libraries(droneInterfaceROSModule ncurses)
#target_link_libraries(parrotInterface_im_dataset_acq ncurses)
#rosbuild_link_boost(  parrotInterface_im_dataset_acq system filesystem)

rosbuild_add_executable(droneInterface_jp_ROSModule ${DRONE_INTERFACE_JP_SOURCE_FILES} ${DRONE_INTERFACE_JP_HEADER_FILES})
target_link_libraries(  droneInterface_jp_ROSModule ncurses)

set(OTHER_FILES
        manifest.xml
        other/keybindings_interfacejp_tentative.txt
)
add_custom_target(other_files SOURCES ${OTHER_FILES})


