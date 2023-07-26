cmake_minimum_required(VERSION 3.0)
project(execute_asap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_msgs
  ff_util
)

catkin_package(
 LIBRARIES
 DEPENDS rospy std_msgs
)

# install_launch_files()

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS scripts/execute_asap.py scripts/chaser_asap.py scripts/target_asap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
)

#############
## Install ##
#############

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)