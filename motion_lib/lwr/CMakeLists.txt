cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

add_definitions(-DNDEBUG)

rosbuild_add_boost_directories() 
rosbuild_add_library(${PROJECT_NAME}  src/lwr.cpp)
rosbuild_add_executable(lwr_regression_test  test/lwr_test.cpp)
target_link_libraries(lwr_regression_test  ${PROJECT_NAME})
