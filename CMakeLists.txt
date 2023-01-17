cmake_minimum_required(VERSION 3.0.2)
project(expansion_board_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Collision.msg
)

add_service_files(
  FILES
  fan_mode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation message_runtime rospy std_msgs
)