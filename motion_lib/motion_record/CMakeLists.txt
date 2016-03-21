cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

rosbuild_add_executable(motion_record src/motion_record.cpp)
rosbuild_add_executable(joint_state_record src/joint_state_record.cpp)
rosbuild_add_executable(controller_state_record src/controller_state_record.cpp)
rosbuild_add_executable(base_controller_state_record src/base_controller_state_record.cpp)
rosbuild_add_executable(pose_odom_state_record src/pose_odom_state_record.cpp)
rosbuild_add_executable(pose_ekf_state_record src/pose_ekf_state_record.cpp)
rosbuild_add_executable(pose_truth_state_record src/pose_truth_state_record.cpp)
