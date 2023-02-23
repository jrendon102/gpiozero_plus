cmake_minimum_required(VERSION 3.0.2)
project(expansion_board_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_python_setup()