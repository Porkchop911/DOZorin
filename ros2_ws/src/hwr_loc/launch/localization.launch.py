"""Localization mode: localizer is the sole map->odom transform owner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    odometry = os.path.join(get_package_share_directory("hwr_odom"),
                            "launch", "odometry.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", description="Saved occupancy-map YAML"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(odometry)),
        Node(package="hwr_loc", executable="localizer_node", name="hwr_localizer",
             parameters=[{"map_yaml": map_yaml}], output="screen"),
    ])
