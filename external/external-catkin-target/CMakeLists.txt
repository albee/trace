cmake_minimum_required(VERSION 3.12)
project(external)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other catkin packages (to be used later)
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
 LIBRARIES
 CATKIN_DEPENDS
)

# QPoases and LAPACK
# if (TRACE_DEBIAN)
#   set(BLAS_LIBRARIES ${ARM_CHROOT_DIR}/usr/lib/libblas/libblas.so ${ARM_CHROOT_DIR}/usr/lib/libblas/libblas.so.3 ${ARM_CHROOT_DIR}/usr/lib/libblas/libblas.so.3.6.0)
#   set(BLA_VENDOR "All")
#   set(LAPACK_LIBRARIES ${ARM_CHROOT_DIR}/usr/lib/liblapack.so.3 ${BLAS_LIBRARIES})

#   message(***************)
#   message(${LAPACK_LIBRARIES})
#   message(${BLAS_LIBRARIES})
#   message(${BLA_VENDOR})
#   message(***************)
# endif (TRACE_DEBIAN)

# set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF)
# set(GTSAM_USE_SYSTEM_EIGEN ON)
# include_directories(${PROJECT_BINARY_DIR}/gtsam ${PROJECT_SOURCE_DIR}/../gtsam)
# # message(WARNING ${PROJECT_SOURCE_DIR}../gtsam/)
# add_subdirectory(../gtsam ${PROJECT_BINARY_DIR}/gtsam/)
# add_subdirectory(../casadi ${PROJECT_BINARY_DIR}/casadi/)
# add_subdirectory(TEASER-plusplus)

# include(ExternalProject)
# ExternalProject_Add(casadi
#     SOURCE_DIR ${PROJECT_SOURCE_DIR}/../casadi
#     BUILD_COMMAND make -j12
#     INSTALL_COMMAND sudo make install
# )

# include(ExternalProject)
# ExternalProject_Add(gtsam
#     SOURCE_DIR ${PROJECT_SOURCE_DIR}/../gtsam
#     CMAKE_ARGS -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON
#     BUILD_COMMAND make -j12
#     INSTALL_COMMAND sudo make install
# )