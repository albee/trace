

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

# Include tests here...
add_executable(conics_test conics_test.cc)
target_link_libraries(conics_test polhode_alignment gtest gtest_main)
add_test(conics conics_test)

add_executable(conic_fitting_test conic_fitting_test.cc)
target_link_libraries(conic_fitting_test polhode_alignment
  gtest gtest_main)
add_test(conic_fitting conic_fitting_test)

add_executable(principal_axes_solver_test principal_axes_solver_test.cc)
target_link_libraries(principal_axes_solver_test polhode_alignment
  gtest gtest_main)
add_test(principal_axes_solver principal_axes_solver_test)

add_executable(rigid_body_rotation_test rigid_body_rotation_test.cc)
target_link_libraries(rigid_body_rotation_test polhode_alignment
  gtest gtest_main)
add_test(rigid_body_rotation rigid_body_rotation_test)
