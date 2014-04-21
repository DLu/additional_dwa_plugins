cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} 
  src/minimize_y_cost_function.cpp
  src/minimize_turns_cost_function.cpp
  src/closest_target_point_map.cpp
  src/path_orientation_cost_function.cpp
  src/goal_orientation_cost_function.cpp
  src/speed_cost_function.cpp
  src/jerk_cost_function.cpp
  src/turn_first_cost_function.cpp
)
