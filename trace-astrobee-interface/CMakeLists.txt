cmake_minimum_required(VERSION 3.0)
project(trace_astrobee_interface)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  ff_msgs diagnostic_msgs
  tf2_geometry_msgs
  actionlib
  msg_conversions
)

catkin_package()
