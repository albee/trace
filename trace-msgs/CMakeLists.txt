cmake_minimum_required(VERSION 3.0)
project(trace_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  trajectory_msgs
  sensor_msgs
)

# equivalent of create_msg_targets
# create_msg_targets(DIR msg SDIR srv
#   DEPS std_msgs geometry_msgs actionlib_msgs trajectory_msgs sensor_msgs)
# Generate messages in the 'msg' folder
file(GLOB MSG_FILES msg/*.msg)
foreach(SRC ${MSG_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND MSG_FILE_NAMES ${NAME})
endforeach()
add_message_files(FILES ${MSG_FILE_NAMES})

# Generate services in the 'srv' folder
file(GLOB SRV_FILES srv/*.srv)
foreach(SRC ${SRV_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND SRV_FILE_NAMES ${NAME})
endforeach()
add_service_files(FILES ${SRV_FILE_NAMES})

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs trajectory_msgs sensor_msgs std_msgs
)

catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs actionlib_msgs trajectory_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
