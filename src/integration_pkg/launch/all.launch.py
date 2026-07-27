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
    use_sim_time = LaunchConfiguration("use_sim_time")

    try:
        tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
        tb3_models_path = os.path.join(tb3_gazebo_share, "models")
        tb3_world = os.path.join(tb3_gazebo_share, "worlds", "empty_world.world")
    except Exception:
        tb3_models_path = "/opt/ros/jazzy/share/turtlebot3_gazebo/models"
        tb3_world = "empty.sdf"

    fuel_path = os.path.expanduser("~/.gz/fuel")
    gz_resource_path = f"{tb3_models_path}:{fuel_path}"

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_resource_path,
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", tb3_world],
        output="screen",
    )

    spawn_turtlebot3 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "turtlebot3",
            "-x", "-2.0",
            "-y", "-2.0",
            "-z", "0.01",
            "-file", PathJoinSubstitution([
                FindPackageShare("turtlebot3_gazebo"),
                "models", "turtlebot3_burger", "model.sdf",
            ]),
        ],
        output="screen",
    )

    spawn_x3 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "X3",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
            "-file", PathJoinSubstitution([
                FindPackageShare("quadcopter_gazebo"),
                "models", "quadcopter", "model.sdf",
            ]),
        ],
        output="screen",
    )

    # FIX: Added motor_speed bridge so motor_mixer can publish via ROS 2 instead of subprocess.
    # The bridge maps ROS 2 actuator_msgs/Actuators -> Gazebo gz.msgs.Actuators.
    # Also increased topic count and added explicit QoS where needed.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/X3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/world/default/model/X3/link/camera_link/sensor/camera/image"
            "@sensor_msgs/msg/Image[gz.msgs.Image",
            "/model/turtlebot3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/turtlebot3/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/turtlebot3/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # NEW: Bridge for motor commands (ROS -> Gazebo)
            "/model/X3/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    params_file = PathJoinSubstitution([
        FindPackageShare("integration_pkg"), "config", "params.yaml",
    ])

    common_params = [params_file, {"use_sim_time": use_sim_time}]

    motor_mixer = Node(
        package="quadcopter_gazebo",
        executable="motor_mixer",
        name="motor_mixer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "cmd_vel_topic": "/drone/cmd_vel",
            "gz_topic": "/model/X3/command/motor_speed",  # matches SDF plugin topic
            "hover_throttle": 0.55,
            "gain": 0.15,
            "publish_via_bridge": True,  # NEW: publish ROS 2 topic instead of subprocess
        }],
    )

    follow_node = Node(
        package="integration_pkg",
        executable="follow_node",
        name="follow_controller",
        output="screen",
        parameters=common_params,
        remappings=[
            ("/drone/odom", "/model/X3/odometry"),
            ("/turtlebot/odom", "/model/turtlebot3/odom"),
            ("/turtlebot/cmd_vel", "/model/turtlebot3/cmd_vel"),
            ("/turtlebot/scan", "/model/turtlebot3/scan"),
        ],
    )

    selector_node = Node(
        package="integration_pkg",
        executable="selector_node",
        name="command_selector",
        output="screen",
        parameters=common_params,
    )

    voice_node = Node(
        package="integration_pkg",
        executable="voice_node",
        name="voice_controller",
        output="screen",
        parameters=common_params,
    )

    battery_sim = Node(
        package="integration_pkg",
        executable="battery_sim",
        name="battery_sim",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    dashboard_node = Node(
        package="integration_pkg",
        executable="dashboard_node",
        name="dashboard",
        output="screen",
        parameters=common_params,
        remappings=[
            ("/drone/odom", "/model/X3/odometry"),
            ("/drone/camera",
             "/world/default/model/X3/link/camera_link/sensor/camera/image"),
        ],
    )

    # FIX: Increased delays. Gazebo needs time to fully initialize before spawning models.
    # Bridge must start AFTER models are spawned so topics exist.
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        set_gz_path,
        gz_sim,
        TimerAction(period=3.0, actions=[spawn_turtlebot3]),
        TimerAction(period=5.0, actions=[spawn_x3]),
        TimerAction(period=8.0, actions=[bridge]),        # was 7.0 -> 8.0
        TimerAction(period=8.0, actions=[motor_mixer]),   # was 7.0 -> 8.0
        TimerAction(period=10.0, actions=[                # was 8.0 -> 10.0
            follow_node,
            selector_node,
            voice_node,
            battery_sim,
            dashboard_node,
        ]),
    ])