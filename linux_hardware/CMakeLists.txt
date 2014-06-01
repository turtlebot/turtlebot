cmake_minimum_required(VERSION 2.8.3)
project(linux_hardware)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs turtlebot_msgs)

catkin_package(CATKIN_DEPENDS turtlebot_msgs diagnostic_msgs)

install(
  PROGRAMS 
    scripts/laptop_battery.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

