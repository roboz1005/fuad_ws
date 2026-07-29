cmake_minimum_required(VERSION 3.8)
project(quad_joystick_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(quad_joystick_interface src/joy.cpp)
ament_target_dependencies(quad_joystick_interface rclcpp geometry_msgs sensor_msgs)

install(TARGETS quad_joystick_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()
