cmake_minimum_required(VERSION 2.8.3)
project(ros_lm_sensors)

find_package(catkin REQUIRED roscpp sensor_msgs diagnostic_updater)

include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_types
  CATKIN_DEPENDS roscpp sensor_msgs diagnostic_updater
  DEPENDS sensors
)

add_library(sensor_types src/sensor_chip.cpp)

add_executable(sensor_monitor src/sensor_monitor.cpp)
target_link_libraries(sensor_monitor sensors sensor_types ${catkin_LIBRARIES})

#############
## Install ##
#############

 install(TARGETS sensor_monitor
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
