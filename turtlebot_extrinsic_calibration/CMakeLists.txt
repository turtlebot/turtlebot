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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Testing code
#rosbuild_add_executable(test/transformation_test test/transformation_test.cpp)
#rosbuild_add_executable(test/interactive_transformation_test test/interactive_transformation_test.cpp)
#rosbuild_add_executable(test/lm_test test/lm_test.cpp)
#rosbuild_add_executable(test/interactive_calibration_test test/interactive_calibration_test.cpp test/estimate_kinect_transform.cpp)
#rosbuild_add_executable(test/point_calibration test/point_calibration.cpp)

rosbuild_add_executable(calibrate_kinect_base_link src/calibrate_kinect_base_link.cpp src/estimate_kinect_link.cpp src/detect_calibration_pattern.cpp )

rosbuild_add_executable(ground_plane_detection src/ground_plane_detection.cpp )

rosbuild_add_executable(calibrate_step_by_step src/calibrate_step_by_step.cpp src/estimate_kinect_link.cpp src/detect_calibration_pattern.cpp src/detect_ground_plane.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
