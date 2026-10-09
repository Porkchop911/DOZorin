"""Mapping mode: odometry plus mapper as the sole map->odom owner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom = os.path.join(get_package_share_directory("hwr_odom"),
                        "launch", "odometry.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("map_path", default_value="hwr_map"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(odom)),
        Node(package="hwr_slam", executable="mapper_node", name="hwr_mapper",
             parameters=[{"map_path": LaunchConfiguration("map_path")}],
             output="screen"),
    ])
