cmake_minimum_required(VERSION 3.0)
project(chaser_coordinator)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other *catkin packages* (to be used later)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_msgs
  ff_util
  ff_common
  msg_conversions
  data
  trace_msgs
)

# Specify catkin info to generate pkg-config and CMake files for *catkin CMake projects* and *other vanilla CMake projects*
catkin_package(
  LIBRARIES
    chaser_coordinator
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    ff_msgs
    ff_util
    ff_common
    geometry_msgs
    msg_conversions
    data
    std_srvs
    trace_msgs
  DEPENDS
)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(chaser_coordinator
  ${CMAKE_CURRENT_SOURCE_DIR}/src/chaser_coordinator_nodelet.cc
)

target_include_directories(chaser_coordinator PUBLIC
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include ${CMAKE_SOURCE_DIR}
  ../data/include/ ../backup_controller/include/
)

target_link_libraries(chaser_coordinator
  ${catkin_LIBRARIES} ${EIGEN_LIBRARIES}
)

# install_launch_files()

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)