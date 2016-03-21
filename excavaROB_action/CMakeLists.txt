cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(left_truck_loading src/left_truck_loading_action.cpp)
rosbuild_add_executable(left_truck_loading_return src/left_truck_loading_return_action.cpp)
rosbuild_add_executable(right_truck_loading src/right_truck_loading_action.cpp)
rosbuild_add_executable(right_truck_loading_return src/left_truck_loading_return_action.cpp)
rosbuild_add_executable(startup src/startup_action.cpp)
rosbuild_add_executable(left_excavation src/left_excavation_action.cpp)
rosbuild_add_executable(right_excavation src/right_excavation_action.cpp)

rosbuild_link_boost(left_truck_loading thread)
rosbuild_link_boost(left_truck_loading_return thread)
rosbuild_link_boost(right_truck_loading thread)
rosbuild_link_boost(right_truck_loading_return thread)
rosbuild_link_boost(startup thread)
rosbuild_link_boost(left_excavation thread)
rosbuild_link_boost(right_excavation thread)
