cmake_minimum_required(VERSION 2.8.3)
project(linux_hardware)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs message_generation rospy std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  LaptopChargeStatus.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package()
