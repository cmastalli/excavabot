cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


rosbuild_init()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_genmsg()
#rosbuild_gensrv()


#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/dmp.cpp
                                     src/transformation_system.cpp
                                     src/canonical_system.cpp
                                     src/trajectory.cpp)
                                     

rosbuild_add_executable(dmp_test test/dmp_test.cpp)
target_link_libraries(dmp_test ${PROJECT_NAME})

