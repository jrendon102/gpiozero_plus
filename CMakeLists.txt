cmake_minimum_required(VERSION 3.0.2)
project(expansion_board_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  fan_mode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation rospy std_msgs
)