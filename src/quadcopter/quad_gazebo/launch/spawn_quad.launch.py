"""Spawn X3 quadcopter in Gazebo Harmonic."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.5"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "X3",
                "-x", LaunchConfiguration("x"),
                "-y", LaunchConfiguration("y"),
                "-z", LaunchConfiguration("z"),
                "-R", LaunchConfiguration("roll"),
                "-P", LaunchConfiguration("pitch"),
                "-Y", LaunchConfiguration("yaw"),
                "-file", PathJoinSubstitution([
                    FindPackageShare("quad_gazebo"),
                    "models", "quadcopter", "model.sdf",
                ]),
            ],
            output="screen",
        ),
    ])
