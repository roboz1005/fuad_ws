cmake_minimum_required(VERSION 3.8)
project(quad_control)

find_package(ament_cmake REQUIRED)

# NOTE: The C++ nodes use mav_msgs and planning_msgs (ROS 1-only).
# For ROS 2 Jazzy + Gazebo Harmonic, the quadcopter is controlled
# via the built-in MulticopterVelocityControl plugin.
# The integration_pkg provides high-level control.
#
# Uncomment below if mav_msgs ROS 2 becomes available.
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(Eigen3 REQUIRED)
# include_directories(include ${EIGEN3_INCLUDE_DIR})
# add_library(quad_controller src/library/quad_controller.cpp)
# ament_target_dependencies(quad_controller rclcpp geometry_msgs nav_msgs sensor_msgs tf2)
# add_executable(waypoint_publisher_node src/nodes/waypoint_publisher_node.cpp)
# ament_target_dependencies(waypoint_publisher_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2)
# target_link_libraries(waypoint_publisher_node quad_controller)
# add_executable(position_controller_node src/nodes/position_controller_node.cpp)
# ament_target_dependencies(position_controller_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2)
# target_link_libraries(position_controller_node quad_controller)
# add_executable(attitude_controller_node src/nodes/attitude_controller_node.cpp)
# ament_target_dependencies(attitude_controller_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2)
# target_link_libraries(attitude_controller_node quad_controller)
# install(TARGETS quad_controller waypoint_publisher_node position_controller_node attitude_controller_node
#   ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
