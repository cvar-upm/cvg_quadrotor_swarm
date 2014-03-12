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

set(CVGUTILSLIB_SOURCE_DIR
        src/source)

set(CVGUTILSLIB_INCLUDE_DIR
        src/include)

set(CVGUTILSLIB_HEADER_FILES
        ${CVGUTILSLIB_INCLUDE_DIR}/matrixLib.h
        ${CVGUTILSLIB_INCLUDE_DIR}/cvg_utils_library.h
        ${CVGUTILSLIB_INCLUDE_DIR}/Timer.h
        ${CVGUTILSLIB_INCLUDE_DIR}/xmlfilereader.h
        ${CVGUTILSLIB_INCLUDE_DIR}/control/FilteredDerivative.h
        ${CVGUTILSLIB_INCLUDE_DIR}/control/LowPassFilter.h
        ${CVGUTILSLIB_INCLUDE_DIR}/control/MidLevelControlledsimMultirotor.h
        ${CVGUTILSLIB_INCLUDE_DIR}/control/PID.h
        ${CVGUTILSLIB_INCLUDE_DIR}/control/Controller_MidLevel_controlModes.h
        ${CVGUTILSLIB_INCLUDE_DIR}/control/simpletrajectorywaypoint.h
        ${CVGUTILSLIB_INCLUDE_DIR}/drone_utils/drone_state_enum.h
        ${CVGUTILSLIB_INCLUDE_DIR}/drone_utils/drone_state_command_enum.h
)

set(CVGUTILSLIB_SOURCE_FILES
        ${CVGUTILSLIB_SOURCE_DIR}/matrixLib.cpp
        ${CVGUTILSLIB_SOURCE_DIR}/cvg_utils_library.cpp
        ${CVGUTILSLIB_SOURCE_DIR}/xmlfilereader.cpp
        ${CVGUTILSLIB_SOURCE_DIR}/control/FilteredDerivative.cpp
        ${CVGUTILSLIB_SOURCE_DIR}/control/LowPassFilter.cpp
        ${CVGUTILSLIB_SOURCE_DIR}/control/MidLevelControlledsimMultirotor.cpp
        ${CVGUTILSLIB_SOURCE_DIR}/control/PID.cpp
)

include_directories(${CVGUTILSLIB_INCLUDE_DIR})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(lib_cvgutils ${CVGUTILSLIB_SOURCE_FILES} ${CVGUTILSLIB_HEADER_FILES})
rosbuild_add_library(lib_pruebastaticbool ${CVGUTILSLIB_INCLUDE_DIR}/pruebastaticbool.h ${CVGUTILSLIB_SOURCE_DIR}/pruebastaticbool.cpp)
target_link_libraries(lib_pruebastaticbool lib_cvgutils)

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})

add_executable(code_tests ${CVGUTILSLIB_SOURCE_DIR}/code_tests.cpp ${CVGUTILSLIB_INCLUDE_DIR}/pruebastaticbool.h)
target_link_libraries(code_tests lib_cvgutils lib_pruebastaticbool)

add_executable(code_tests2 ${CVGUTILSLIB_SOURCE_DIR}/code_tests2.cpp ${CVGUTILSLIB_INCLUDE_DIR}/pruebastaticbool.h)
target_link_libraries(code_tests2 lib_cvgutils lib_pruebastaticbool)

