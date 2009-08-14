cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(wam_ros)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#uncomment if you have defined messages
genmsg()
#uncomment if you have defined services
#gensrv()
#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rospack_add_library(wam_ros ../../bindings/cpp/BtWam.cpp)
rospack_add_library(wam_ros ../../bindings/cpp/Mutex.hpp)
rospack_add_library(wam_ros ../../bindings/cpp/Posix.cpp)

rospack_add_executable(WamJpos src/WamJpos.cpp)
target_link_libraries(WamJpos wam_ros)
target_link_libraries(WamJpos -lbarrett)

rospack_add_executable(WamState src/WamState.cpp)
target_link_libraries(WamState wam_ros)
target_link_libraries(WamState -lbarrett)

rospack_add_executable(WamViewer src/WamViewer.cpp)
target_link_libraries(WamViewer wam_ros)
target_link_libraries(WamViewer -lbarrett) # barrett libbarrett -lbarrett
target_link_libraries(WamViewer -lcurses)

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(LIBBARRETT libbarrett)
#nclude_directories(${LIBBARRETT_INCLUDE_DIRS})
#link_directories(${LIBBARRETT_LIBRARY_DIRS})
#include_directories(/usr/include)
#link_directories(/usr/lib)
#target_link_libraries(WamJpos ${LIBBARRETT_LIBRARIES})

