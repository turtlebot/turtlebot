cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmd_vel_mux geometry_msgs joy kobuki_safety_controller roscpp yocs_velocity_smoother)

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlebot_teleop
  CATKIN_DEPENDS cmd_vel_mux geometry_msgs joy kobuki_safety_controller roscpp yocs_velocity_smoother
  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(turtlebot_teleop_joy src/turtlebot_joy.cpp)
target_link_libraries(turtlebot_teleop_joy ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/turtlebot_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS turtlebot_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

