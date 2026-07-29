cmake_minimum_required(VERSION 3.8)
project(moveit_simple_controller_manager)

find_package(ament_cmake REQUIRED)

# NOTE: This package is a ROS 1 MoveIt plugin. Not compiled in ROS 2.

install(DIRECTORY include/ DESTINATION include)

ament_package()
