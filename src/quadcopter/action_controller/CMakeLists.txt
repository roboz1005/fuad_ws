cmake_minimum_required(VERSION 3.8)
project(action_controller)

find_package(ament_cmake REQUIRED)

# NOTE: This package depends on ROS 1 actionlib and moveit_core.
# Preserved for reference but not compiled in ROS 2.

install(DIRECTORY action/ DESTINATION share/${PROJECT_NAME}/action)

ament_package()
