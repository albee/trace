cmake_minimum_required(VERSION 3.0)
project(polhode_alignment)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other catkin packages (to be used later)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
 LIBRARIES
 CATKIN_DEPENDS
  std_msgs
)

# External package dependencies.
#find_package(Eigen3 3.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set(SRC_DIR "src/")
set(PAO_SRCS
  ${SRC_DIR}/conic_fitting.cc
  ${SRC_DIR}/rigid_body_rotation.cc
  )
set(PAO_HDRS
  )
set(PAO_LIBS
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES})

add_library(polhode_alignment
  ${PAO_SRCS} ${PAO_HDRS}
)
target_include_directories(polhode_alignment PUBLIC
  ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${CMAKE_CURRENT_SOURCE_DIR}/../spheres-vertigo/include/
)
target_link_libraries(polhode_alignment PUBLIC 
  ${PAO_LIBS}
)
target_compile_options(polhode_alignment PRIVATE -Wall -Wextra -pedantic -Werror)

# Make library reachable by other cmake projects.
export(PACKAGE polhode_alignment)
export(TARGETS FILE polhode-alignmentConfig.cmake)
# Build a bunch of useful examples.
add_subdirectory(examples)
# Include unit tests directory to the project.
enable_testing()
add_subdirectory(tests)
