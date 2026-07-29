"""Launch Gazebo Harmonic with quadcopter world."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare("quad_gazebo"),
        "worlds", "quad_basic.world",
    ])

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
    )

    spawn_quad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("quad_gazebo"),
                "launch", "spawn_quad.launch.py",
            ])
        ]),
    )

    return LaunchDescription([
        gz_sim,
        TimerAction(period=3.0, actions=[spawn_quad]),
    ])
