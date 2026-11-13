"""RViz mirror of the real M0609 while the DRFL GUI controls it (read-only, no control authority).
  ros2 launch m0609_mirror mirror.launch.py [ip:=192.168.137.100] [rviz:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("dsr_description2"), "xacro", "m0609.urdf.xacro")
    rviz_cfg = os.path.join(get_package_share_directory("m0609_mirror"), "rviz", "mirror.rviz")
    desc = ParameterValue(Command(["xacro ", xacro_file, " color:=white"]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("ip", default_value="192.168.137.100"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="m0609_mirror", executable="drfl_mirror", output="screen",
             parameters=[{"ip": LaunchConfiguration("ip"), "rate": 50.0}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": desc}]),
        Node(package="m0609_mirror", executable="scene_markers.py", output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
