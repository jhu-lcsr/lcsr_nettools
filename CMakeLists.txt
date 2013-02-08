cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/statistics.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_subdirectory(rviz_plugins)

rosbuild_add_executable(statistics_node src/statistics_node.cpp)
target_link_libraries(statistics_node lcsr_nettools)

# Statistics python wrapper
#find_package(PythonLibs REQUIRED)

#rosbuild_add_boost_directories()
#rosbuild_add_library(statistics src/statistics_py.cpp)
#rosbuild_link_boost(statistics python)     
#include_directories(${PYTHON_INCLUDE_DIRS})
#target_link_libraries(statistics lcsr_nettools ${PYTHON_LIBRARIES}) 
#rosbuild_add_executable(header_viewer src/header_viewer.cpp)

