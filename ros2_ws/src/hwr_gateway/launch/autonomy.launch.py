"""Saved-map autonomy graph, including arm behaviors and authenticated gateway."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rosbridge():
    """rosbridge :9090 + rosapi for the Hermes ros MCP; optional so a robot without it still boots."""
    try:
        share = get_package_share_directory("rosbridge_server")
    except PackageNotFoundError:
        return LogInfo(msg="rosbridge_server not installed; ros MCP unavailable")
    return IncludeLaunchDescription(AnyLaunchDescriptionSource(
        os.path.join(share, "launch", "rosbridge_websocket_launch.xml")))


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
        rosbridge(),
    ])
