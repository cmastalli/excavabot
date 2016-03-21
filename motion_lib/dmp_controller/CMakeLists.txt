cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


rosbuild_init()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_genmsg()
rosbuild_gensrv()


#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/dmp_controller.cpp
				     src/dmp_ik_controller.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)

#---
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#---

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#target_link_libraries(${PROJECT_NAME} dynamic_movement_primitives)
#rosbuild_remove_compile_flags(${PROJECT_NAME} -W)

