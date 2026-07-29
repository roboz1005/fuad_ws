cmake_minimum_required(VERSION 3.8)
project(quad_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
