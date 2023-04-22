cmake_minimum_required(VERSION 3.0)
project(motion_planner_interface)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other catkin packages (to be used later)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  actionlib
  tf
  geometry_msgs
  std_msgs
  message_generation
  message_runtime
  ff_util
  trace_msgs
  ff_msgs
)

add_message_files(
   FILES
   mpdebug.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES
    motion_planner_interface
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    geometry_msgs
    std_msgs
    message_generation
    message_runtime
    ff_util
    trace_msgs
    ff_msgs
)

find_package(Eigen3 REQUIRED)

add_library(motion_planner_interface
  ${CMAKE_CURRENT_SOURCE_DIR}/../data/src/traj_utils.cc
)

target_include_directories(motion_planner_interface PUBLIC
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include ${CMAKE_CURRENT_SOURCE_DIR}/../data/include
)

target_link_libraries(motion_planner_interface
  ${catkin_LIBRARIES} ${EIGEN_LIBRARIES}
)
