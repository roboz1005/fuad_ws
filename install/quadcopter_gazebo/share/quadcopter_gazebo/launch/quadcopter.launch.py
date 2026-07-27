"""
FILE LOCATION: fuad_ws/src/quadcopter_gazebo/launch/quadcopter.launch.py

Spawns the local quadcopter model into an existing Gazebo world.
Also starts the motor mixer that bridges /drone/cmd_vel -> Gazebo rotor speeds.

USAGE (standalone):
    ros2 launch quadcopter_gazebo quadcopter.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Spawn the local quadcopter model, which includes odometry and camera topics.
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

    # Motor mixer: converts Twist -> rotor speeds for X3
    motor_mixer = Node(
        package="quadcopter_gazebo",
        executable="motor_mixer",
        name="motor_mixer",
        output="screen",
        parameters=[{
            "cmd_vel_topic": "/drone/cmd_vel",
            "gz_topic": "/model/X3/gazebo/command/motor_speed",
            "hover_throttle": 0.55,
            "gain": 0.15,
        }],
    )

    return LaunchDescription([
        spawn_x3,
        motor_mixer,
    ])
