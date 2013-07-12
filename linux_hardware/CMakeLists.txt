cmake_minimum_required(VERSION 2.8.3)
project(linux_hardware)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES LaptopChargeStatus.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs diagnostic_msgs)

install(
  PROGRAMS 
    scripts/laptop_battery.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

