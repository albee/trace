cmake_minimum_required(VERSION 3.0)
project(mit_slam)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other *catkin packages* (to be used later)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_msgs
  trace_astrobee_interface
  msg_conversions
  data
  polhode_alignment
  tf
  tf2
  trace_msgs
  # teaserpp
)

# Specify catkin info to generate pkg-config and CMake files for *catkin CMake projects* and *other vanilla CMake projects*
catkin_package(
 LIBRARIES
  mit_slam
  ${CERES_LIBRARIES}
 DEPENDS
  # teaserpp
 CATKIN_DEPENDS
  roscpp
  sensor_msgs
  tf
  tf2
  std_msgs
  geometry_msgs
  nodelet
  pluginlib
  actionlib
  ff_msgs
  polhode_alignment
  data
  trace_msgs
)

# set(GTSAM_DIR "${PROJECT_BINARY_DIR}/../external/gtsam/")
# message(WARNING ${GTSAM_DIR})
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL QUIET REQUIRED COMPONENTS common features kdtree filters)
find_package(Boost 1.58 COMPONENTS timer REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Ceres REQUIRED)

if (NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS} 
)

# Get TEASER++ to be recognized.
# set(teaserpp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../external/TEASER-plusplus/cmake")
find_package(teaserpp REQUIRED COMPONENTS teaserpp::teaser_registration teaserpp::teaser_features)
message(STATUS ${teaserpp_DIR})

file(GLOB_RECURSE SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
)

add_library(mit_slam
  ${SOURCE_FILES}
)

target_include_directories(mit_slam PUBLIC
  include ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} ${teaserpp_INCLUDE_DIRS} gtsam polhode-alignment
  ${CMAKE_CURRENT_SOURCE_DIR}/../polhode-alignment/include/ ${CMAKE_CURRENT_SOURCE_DIR}/../spheres-vertigo/include/
)

target_link_libraries(mit_slam
${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} gtsam teaserpp::teaser_registration teaserpp::teaser_features
)

# install_launch_files()

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)