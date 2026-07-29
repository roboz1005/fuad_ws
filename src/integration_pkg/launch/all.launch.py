"""
Master launch file: X3 quadcopter + TurtleBot3 in Gazebo Harmonic.

Usage:
  ros2 launch integration_pkg all.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Package directories
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    tb3_models_path = os.path.join(tb3_gazebo_share, "models")
    tb3_world = os.path.join(tb3_gazebo_share, "worlds", "empty_world.world")

    quad_share = get_package_share_directory("quad_gazebo")
    quad_model_path = os.path.join(quad_share, "models")

    # Gazebo resource paths
    fuel_path = os.path.expanduser("~/.gz/fuel")
    gz_resource_path = f"{tb3_models_path}:{quad_model_path}:{fuel_path}"

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_resource_path,
    )

    # Start Gazebo Harmonic
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", tb3_world],
        output="screen",
    )

    # Spawn X3 Quadcopter
    spawn_x3 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "X3",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
            "-file", PathJoinSubstitution([
                FindPackageShare("quad_gazebo"),
                "models", "quadcopter", "model.sdf",
            ]),
        ],
        output="screen",
    )

    # Spawn TurtleBot3 Burger
    spawn_turtlebot3 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "turtlebot3",
            "-x", "-2.0",
            "-y", "-2.0",
            "-z", "0.01",
            "-Y", "0.0",
            "-file", PathJoinSubstitution([
                FindPackageShare("turtlebot3_gazebo"),
                "models", "turtlebot3_burger", "model.sdf",
            ]),
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # X3 Quadcopter
            "/model/X3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/X3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/world/default/model/X3/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/default/model/X3/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # TurtleBot3
            "/model/turtlebot3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/turtlebot3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/turtlebot3/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/turtlebot3/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # Clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # Integration nodes
    selector_node = Node(
        package="integration_pkg",
        executable="selector_node",
        name="command_selector",
        parameters=[os.path.join(
            get_package_share_directory("integration_pkg"),
            "config", "params.yaml"
        )],
        output="screen",
    )

    # TF broadcaster — uses odometry topics, no extra bridge needed
    tf_broadcaster = Node(
        package="integration_pkg",
        executable="tf_broadcaster",
        name="gazebo_tf_broadcaster",
        parameters=[{
            "odom_topics": ["/model/X3/odometry", "/model/turtlebot3/odometry"],
            "frame_ids": ["X3/base_link", "turtlebot3/base_link"],
        }],
        output="screen",
    )

    # TF follow node (bonus — 15 pts)
    tf_follow_node = Node(
        package="integration_pkg",
        executable="tf_follow_node",
        name="tf_follow_controller",
        parameters=[os.path.join(
            get_package_share_directory("integration_pkg"),
            "config", "params.yaml"
        )],
        output="screen",
    )

    voice_node = Node(
        package="integration_pkg",
        executable="voice_node",
        name="voice_controller",
        parameters=[os.path.join(
            get_package_share_directory("integration_pkg"),
            "config", "params.yaml"
        )],
        output="screen",
    )

    battery_sim = Node(
        package="integration_pkg",
        executable="battery_sim",
        name="battery_sim",
        parameters=[os.path.join(
            get_package_share_directory("integration_pkg"),
            "config", "params.yaml"
        )],
        output="screen",
    )

    # Dashboard GUI
    dashboard_node = Node(
        package="integration_pkg",
        executable="dashboard_node",
        name="dashboard",
        parameters=[os.path.join(
            get_package_share_directory("integration_pkg"),
            "config", "params.yaml"
        )],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        set_gz_path,
        gz_sim,
        # Spawn robots after Gazebo starts
        TimerAction(period=3.0, actions=[spawn_x3]),
        TimerAction(period=5.0, actions=[spawn_turtlebot3]),
        # Start bridge after robots spawn
        TimerAction(period=7.0, actions=[bridge]),
        # Start integration nodes after bridge is up
        TimerAction(period=9.0, actions=[
            selector_node,
            voice_node,
            battery_sim,
            tf_broadcaster,
            tf_follow_node,
            dashboard_node,
        ]),
    ])
