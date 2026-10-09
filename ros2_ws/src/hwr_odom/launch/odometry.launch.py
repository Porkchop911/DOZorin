"""Phase-5 full graph with fused odometry as the sole odom TF owner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(package, filename, arguments=None):
    path = os.path.join(get_package_share_directory(package), "launch", filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(arguments or {}).items())


def generate_launch_description():
    return LaunchDescription([
        _include("hwr_base", "bringup.launch.py", {"model_tf": "false"}),
        _include("hwr_sensors", "sensors.launch.py"),
        Node(package="hwr_odom", executable="odom_node", name="hwr_odom",
             output="screen"),
    ])
