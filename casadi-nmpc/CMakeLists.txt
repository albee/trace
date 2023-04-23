cmake_minimum_required(VERSION 3.0)
project(casadi_nmpc)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other *catkin packages* (to be used later)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_msgs
  ff_util
  data
  trace_msgs
)

# Specify catkin info to generate pkg-config and CMake files for *catkin CMake projects* and *other vanilla CMake projects*
catkin_package(
  LIBRARIES
    casadi_nmpc
  CATKIN_DEPENDS
    roscpp
    nodelet
    ff_msgs
    ff_util
    data
    trace_msgs
)

find_package(Boost 1.58 COMPONENTS timer REQUIRED)

# codegen is not required when using the serial interface
#add_subdirectory(casadi_codegen)

# System dependencies found via normal CMake
find_package(Eigen3 REQUIRED)
# find_package(CasADi REQUIRED)
find_package(PkgConfig)
pkg_search_module(CASADI casadi)
include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#---casadi_nmpc---
# Astrobee-esque way of linking libraries and adding include directories. All files in /src get added to TARGET by default.
# NB1: CMAKE_CURRENT_LIST_DIR will get the current directory of CMakeLists.txt, rather than just the top level
# NB2: must specify FULL shared library name for linking to work this way
# NB3: casadi is installed from source by the local user. See casadi_nmpc/README.md.
# create_library(TARGET casadi_nmpc
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${CASADI_LIBRARIES} ff_common ff_nodelet msg-conversions ff_flight casadi_robust_tube_mpc casadi_mpc # these libraries are created elsewhere in the CMake hierarchy
#   INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CASADI_INCLUDEDIR} ${CMAKE_CURRENT_SOURCE_DIR}/../data/include ${CMAKE_CURRENT_SOURCE_DIR}../backup_controller/include
#   DEPS ff_msgs ff_hw_msgs casadi_nmpc_export
#   ADD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/../data/src/csv_read.cc ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/src/backup_controller.cc) # additional source files

### Codegen version
# create_library(TARGET casadi_nmpc
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} casadi ff_common ff_nodelet msg-conversions ff_flight casadi_robust_tube_mpc casadi_mpc
#   INC casadi ${CMAKE_CURRENT_SOURCE_DIR}/../motion_planner_interface/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../data/include ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/include
#   DEPS ff_msgs ff_hw_msgs
#   ADD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/../data/src/csv_read.cc ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/src/backup_controller.cc)

### Serialized version
# create_library(TARGET casadi_nmpc
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} casadi ff_common ff_nodelet msg-conversions ff_flight csv_read
#   INC casadi include ${CMAKE_CURRENT_SOURCE_DIR}/../motion_planner_interface/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../data/include ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/include
#   DEPS ff_msgs ff_hw_msgs
#   ADD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/src/backup_controller.cc)

add_library(casadi_nmpc src/casadi_nmpc_nodelet.cc src/casadi_nmpc_utils.cc src/casadi_nmpc_ros.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/../backup-controller/src/backup_controller.cc)

target_link_libraries(casadi_nmpc 
  ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} casadi
)

target_include_directories(casadi_nmpc PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include casadi
  ${CMAKE_CURRENT_SOURCE_DIR}/../motion-planner-interface/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../data/include 
  ${CMAKE_CURRENT_SOURCE_DIR}/../backup-controller/include)

add_dependencies(casadi_nmpc
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS casadi_nmpc DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.tpp")

# install_launch_files()

install(DIRECTORY scripts/pytope/ DESTINATION share/${PROJECT_NAME}/scripts/pytope)
install(PROGRAMS scripts/z_poly_calc.py DESTINATION share/${PROJECT_NAME}/scripts PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE)
install(PROGRAMS scripts/service_call_tester.py DESTINATION share/${PROJECT_NAME}/scripts PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE)

# Enable gtest unit tests
enable_testing()
add_subdirectory(tests)
