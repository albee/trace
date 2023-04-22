cmake_minimum_required(VERSION 3.5)
project(sph-vertigo)

# Set some compilation options.
set(CMAKE_CXX_STANDARD 11)

# External package dependencies.
find_package(Eigen3 REQUIRED)
# find_package(cpp-toolbox REQUIRED)
# find_package(Pangolin REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
# find_package(SESync REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

# Create library.
set(VERTIGO_SRCS
  #src/RotationKinematicFactor.cpp       # GTSAM factor.
  #src/ConicProjectionFactor.cpp         # GTSAM factor.
  #src/InertiaRatiosFactor.cpp           # GTSAM factor.
  #src/InertiaRatiosManifoldFactor.cpp   # GTSAM factor.
  #src/BetweenMomentumFactor.cpp         # GTSAM factor.
  #src/InertiaRatios.cpp                 # GTSAM manifold.
  #src/InertiaRatiosOpt.cpp              # Offline opt.
  #src/PrincipalAxesOpt.cpp              # Offline opt.
  # Removed src/PrincipalAxesOpt.cpp because we've templated it.
  #src/Conics.cpp                        # Offline opt.
  # Removed src/Conics.cpp because we've templated it.
  src/RigidBodyRotation.cpp             # For data simulation.
  src/CommonTypes.cpp
  src/DataUtils.cpp
  src/GeometryUtils.cpp
  #src/Visualizer.cpp
  #src/ImageUtils.cpp
  #src/FactorUtils.cpp
  #src/FeatureManager.cpp
  #src/FeatureHandler.cpp
  #src/Rectifier.cpp
  #src/GraphManager.cpp
  #src/FrontEnd.cpp
  #src/VisualOdometer.cpp
  #src/StereoDepthSensor.cpp
  # src/InertialOdometer.cpp
  )

set(VERTIGO_LIBS
  ${Boost_LIBRARIES}
  Eigen3
  ${CERES_LIBRARIES})

add_library(sph-vertigo STATIC ${VERTIGO_SRCS})
target_include_directories(sph-vertigo PUBLIC include ${CMAKE_CURRENT_SOURCE_DIR}/../polhode-alignment/src/ ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(sph-vertigo PUBLIC ${VERTIGO_LIBS})
target_compile_options(sph-vertigo PRIVATE -Wall)

# Make library reachable by other cmake projects.
export(PACKAGE sph-vertigo)
export(TARGETS sph-vertigo FILE sph-vertigoConfig.cmake)

# Build a bunch of useful examples.
# add_subdirectory(examples)

# Include unit tests directory to the project.
enable_testing()
# add_subdirectory(tests)
