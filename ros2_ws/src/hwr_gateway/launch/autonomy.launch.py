"""Saved-map autonomy graph, including arm behaviors and authenticated gateway."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation = os.path.join(get_package_share_directory("hwr_nav"),
                              "launch", "navigation.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(navigation),
                                 launch_arguments={"map_yaml":
                                                   LaunchConfiguration("map_yaml")}.items()),
        Node(package="hwr_gateway", executable="gateway_node", name="hwr_gateway",
             output="screen"),
        Node(package="hwr_gateway", executable="vision_node", name="hwr_vision",
             output="screen"),
    ])
