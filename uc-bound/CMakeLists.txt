cmake_minimum_required(VERSION 3.0)
project(uc_bound)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other catkin packages (to be used later)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  actionlib
  tf
  ff_msgs
  std_msgs
  geometry_msgs
  ff_util
  ff_common
  eigen_conversions
  trace_msgs
)

catkin_package(
  LIBRARIES
    uc_bound
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    ff_msgs
    std_msgs
    geometry_msgs
    ff_util
    ff_common
    eigen_conversions
    trace_msgs

)

find_package(Eigen3 REQUIRED)

add_library(uc_bound
  ${CMAKE_CURRENT_SOURCE_DIR}/src/uc_bound_nodelet.cc
)

target_include_directories(uc_bound PUBLIC
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include ${CMAKE_SOURCE_DIR}
  ../data/include/
)

target_link_libraries(uc_bound
  ${catkin_LIBRARIES} ${EIGEN_LIBRARIES}
)

# install_launch_files()
