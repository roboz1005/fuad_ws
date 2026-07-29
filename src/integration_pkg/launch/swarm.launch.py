"""
Swarm launch: N TurtleBot3 followers + Quadcopter leader.

Usage:
  ros2 launch integration_pkg swarm.launch.py num_tb3:=3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    num_tb3 = int(LaunchConfiguration("num_tb3").perform(context))

    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    tb3_models_path = os.path.join(tb3_gazebo_share, "models")
    tb3_world = os.path.join(tb3_gazebo_share, "worlds", "empty_world.world")

    quad_share = get_package_share_directory("quad_gazebo")
    quad_model_path = os.path.join(quad_share, "models")

    fuel_path = os.path.expanduser("~/.gz/fuel")
    gz_resource_path = f"{tb3_models_path}:{quad_model_path}:{fuel_path}"

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_resource_path,
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", tb3_world],
        output="screen",
    )

    actions = [set_gz_path, gz_sim]

    # Spawn quadcopter leader
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
    actions.append(TimerAction(period=3.0, actions=[spawn_x3]))

    bridge_topics = [
        # X3 Quadcopter
        "/model/X3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/model/X3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        # Clock
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ]

    # Build lists for tf_broadcaster
    odom_topics = ["/model/X3/odometry"]
    frame_ids = ["X3/base_link"]

    # Spawn N TurtleBot3s
    for i in range(num_tb3):
        ns = f"tb3_{i+1}"
        x = -2.0 - (i * 1.5)
        y = -2.0

        spawn_tb3 = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", ns,
                "-x", str(x),
                "-y", str(y),
                "-z", "0.01",
                "-Y", "0.0",
                "-file", PathJoinSubstitution([
                    FindPackageShare("turtlebot3_gazebo"),
                    "models", "turtlebot3_burger", "model.sdf",
                ]),
            ],
            output="screen",
        )
        actions.append(TimerAction(period=5.0 + i * 1.0, actions=[spawn_tb3]))

        bridge_topics.extend([
            f"/model/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/model/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/model/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ])

        odom_topics.append(f"/model/{ns}/odometry")
        frame_ids.append(f"{ns}/base_link")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_topics,
        output="screen",
    )
    actions.append(TimerAction(period=7.0 + num_tb3 * 1.0, actions=[bridge]))

    # TF broadcaster for all models
    tf_broadcaster = Node(
        package="integration_pkg",
        executable="tf_broadcaster",
        name="gazebo_tf_broadcaster",
        parameters=[{
            "odom_topics": odom_topics,
            "frame_ids": frame_ids,
        }],
        output="screen",
    )
    actions.append(TimerAction(period=8.0 + num_tb3 * 1.0, actions=[tf_broadcaster]))

    # TF follow nodes for each TB3
    for i in range(num_tb3):
        ns = f"tb3_{i+1}"
        tf_follow = Node(
            package="integration_pkg",
            executable="tf_follow_node",
            name=f"tf_follow_{ns}",
            parameters=[{
                "tb3_cmd_vel_topic": f"/model/{ns}/cmd_vel",
                "tb3_scan_topic": f"/model/{ns}/scan",
                "follower_frame": f"{ns}/base_link",
                "leader_frame": "X3/base_link",
                "follow_distance": 1.5 + i * 0.5,
            }],
            output="screen",
        )
        actions.append(TimerAction(period=9.0 + num_tb3 * 1.0, actions=[tf_follow]))

    # Core control nodes
    selector_node = Node(
        package="integration_pkg",
        executable="selector_node",
        name="command_selector",
        output="screen",
    )
    voice_node = Node(
        package="integration_pkg",
        executable="voice_node",
        name="voice_controller",
        output="screen",
    )
    battery_sim = Node(
        package="integration_pkg",
        executable="battery_sim",
        name="battery_sim",
        output="screen",
    )
    dashboard_node = Node(
        package="integration_pkg",
        executable="dashboard_node",
        name="dashboard",
        output="screen",
    )
    actions.append(TimerAction(period=11.0 + num_tb3 * 1.0, actions=[
        selector_node, voice_node, battery_sim, dashboard_node
    ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "num_tb3",
            default_value="2",
            description="Number of TurtleBot3 followers",
        ),
        OpaqueFunction(function=launch_setup),
    ])
