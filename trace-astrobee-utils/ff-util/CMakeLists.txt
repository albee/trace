cmake_minimum_required(VERSION 3.0)
project(ff_util)

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
  ff_msg_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ff_nodelet
  CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure ff_msgs diagnostic_msgs tf2_geometry_msgs actionlib ff_msg_conversions
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(ff_nodelet
  src/ff_nodelet/ff_nodelet.cc
)
add_dependencies(ff_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(ff_nodelet ${catkin_LIBRARIES})


#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ff_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
