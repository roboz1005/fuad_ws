cmake_minimum_required(VERSION 3.8)
project(quad_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource)

install(DIRECTORY materials/
  DESTINATION share/${PROJECT_NAME}/materials)

ament_package()
