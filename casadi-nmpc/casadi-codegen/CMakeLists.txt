# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the TBD
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

project(casadi_codegen)

message("-- Add CasADI MPC codegen .so's...")

set(SOURCE_FILES_MPC casadi_mpc.c)
set(SOURCE_FILES_TUBE_MPC casadi_robust_tube_mpc.c)

add_library(casadi_mpc SHARED
  ${SOURCE_FILES_MPC}
)
add_library(casadi_robust_tube_mpc SHARED
  ${SOURCE_FILES_TUBE_MPC}
)

target_include_directories(casadi_mpc PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(casadi_robust_tube_mpc PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(casadi_mpc m)  # math library...not linked by default?
target_link_libraries(casadi_robust_tube_mpc m )  # math library...not linked by default?
# target_link_libraries(casadi_robust_tube_mpc PRIVATE osqp::osqp m )  # math library...not linked by default?

install(TARGETS casadi_mpc DESTINATION lib)
install(TARGETS casadi_robust_tube_mpc DESTINATION lib)
