"""Localization/navigation mode; mapper is deliberately absent."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom = os.path.join(get_package_share_directory("hwr_odom"),
                        "launch", "odometry.launch.py")
    map_yaml = LaunchConfiguration("map_yaml")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(odom)),
        Node(package="hwr_arm", executable="arm_node", name="hwr_arm",
             output="screen"),
        Node(package="hwr_loc", executable="localizer_node", name="hwr_localizer",
             parameters=[{"map_yaml": map_yaml}], output="screen"),
        Node(package="hwr_nav", executable="nav_node", name="hwr_nav",
             parameters=[{"map_yaml": map_yaml}], output="screen"),
    ])
