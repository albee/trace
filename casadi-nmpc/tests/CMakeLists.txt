# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

# Include tests here...
add_executable(casadi_nmpc_test casadi_nmpc_test.cc)
target_link_libraries(casadi_nmpc_test casadi_nmpc gtest gtest_main)
target_include_directories(casadi_nmpc_test PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../backup-controller/include)
add_test(casadi_nmpc casadi_nmpc_test)