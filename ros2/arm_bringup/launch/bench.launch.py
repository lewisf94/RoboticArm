"""One command to bring up the physical bench arm with a live RViz mirror.

    ros2 launch arm_bringup bench.launch.py
    ros2 launch arm_bringup bench.launch.py port:=/dev/ttyACM1
    ros2 launch arm_bringup bench.launch.py rviz:=false

Chain: arm_bridge reads the device and publishes /joint_states ->
robot_state_publisher turns those angles into TF using arm_description's
URDF -> RViz draws it. Note there is deliberately no joint_state_publisher
here (unlike arm_description's view.launch.py): the device is the source of
joint angles now, and running both would fight over /joint_states.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("arm_description")
    xacro_path = os.path.join(description_share, "urdf", "bench_arm.urdf.xacro")
    rviz_config = os.path.join(description_share, "rviz", "arm.rviz")

    robot_description = xacro.process_file(xacro_path).toxml()

    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyACM0",
                description="Serial device the ESP32 enumerates as.",
            ),
            DeclareLaunchArgument(
                "baud", default_value="115200", description="Serial baud rate."
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Set false for a headless bringup (no RViz).",
            ),
            Node(
                package="arm_bridge",
                executable="bridge",
                name="arm_bridge",
                output="screen",
                parameters=[{"port": port, "baud": baud}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
