cmake_minimum_required(VERSION 3.0)
project(data)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other *catkin packages* (to be used later)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ff_msgs
  rosbag
)

catkin_package(
  LIBRARIES
    traj_utils
    csv_read
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    ff_msgs
    rosbag
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#---traj_utils---
add_library(traj_utils
  ${CMAKE_CURRENT_SOURCE_DIR}/src/traj_utils.cc
)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(traj_utils
  ${catkin_LIBRARIES}
)  # make sure ROS .so's are found


#---read_std_vec_bag---
add_executable(read_std_vec_bag
  ${CMAKE_CURRENT_SOURCE_DIR}/src/read_std_vec_bag.cc
)
target_link_libraries(read_std_vec_bag
  ${catkin_LIBRARIES}
)
include_directories(include
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}
)
link_directories(${Boost_LIBRARY_DIRS})


#---csv_read (still used by Target!)---
add_library(csv_read
  ${CMAKE_CURRENT_SOURCE_DIR}/src/csv_read.cc
)
include_directories(include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(csv_read
  ${catkin_LIBRARIES}
)  # make sure ROS .so's are found

install(DIRECTORY input/sample-trajectories/ DESTINATION share/${PROJECT_NAME}/input)
install(FILES input/target-tumbles/smooth_omega_test6_flipped.csv DESTINATION /opt/roam/mp)
install(TARGETS traj_utils DESTINATION bin)
install(DIRECTORY input/ DESTINATION share/${PROJECT_NAME}/input)
