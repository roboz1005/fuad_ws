cmake_minimum_required(VERSION 3.8)
project(quad_3dnav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
