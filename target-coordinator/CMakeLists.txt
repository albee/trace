cmake_minimum_required(VERSION 3.0)
project(target_coordinator)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other catkin packages (to be used later)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_msgs
  trace_astrobee_interface
  msg_conversions
  data
  trace_msgs
)

# Specify catkin info to generate pkg-config and CMake files
catkin_package(
  LIBRARIES
    target_coordinator
  DEPENDS
  CATKIN_DEPENDS  # other catkin deps (must go in package.xml!)
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    ff_msgs
    trace_astrobee_interface
    geometry_msgs
    msg_conversions
    sensor_msgs
    nav_msgs
    data
    trace_msgs
    std_srvs
)

add_library(target_coordinator
  ${CMAKE_CURRENT_SOURCE_DIR}/src/target_coordinator_nodelet.cc
)

target_include_directories(target_coordinator PUBLIC
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include ${CMAKE_SOURCE_DIR}
  ../data/include/
)

target_link_libraries(target_coordinator
  ${catkin_LIBRARIES} ${EIGEN_LIBRARIES}
)

# install_launch_files()
