project(backup_controller)

catkin_package(
  LIBRARIES
    backup_controller
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    nodelet
    ff_util
    ff_common
    pluginlib
    actionlib
    tf
    ff_msgs
)

add_library(backup_controller
  src/backup_controller.cc
)
target_link_libraries(backup_controller
  ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ff_common ff_nodelet msg_conversions ff_flight
)
target_include_directories(backup_controller PUBLIC
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include
)
add_dependencies(backup_controller
  ff_msgs ff_hw_msgs
)

#add_executable(fake_setpoint src/fake_setpoint.cc)
#target_include_directories(fake_setpoint PUBLIC ${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
#target_link_libraries(fake_setpoint ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} )
#add_dependencies(fake_setpoint
#        ${${PROJECT_NAME}_EXPORTED_TARGETS}
#         ${catkin_EXPORTED_TARGETS} )

find_package(Eigen3 REQUIRED)

# install_launch_files()

install(TARGETS backup_controller DESTINATION lib)
