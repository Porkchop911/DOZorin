import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping = os.path.join(get_package_share_directory("hwr_slam"),
                           "launch", "mapping.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("map_path", default_value="downstairs_map"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping),
            launch_arguments={"map_path": LaunchConfiguration("map_path")}.items()),
        Node(package="hwr_arm", executable="arm_node", name="hwr_arm",
             output="screen"),
        Node(package="hwr_nav", executable="nav_node", name="hwr_nav",
             parameters=[{"map_yaml": ""}], output="screen"),
        Node(package="hwr_explore", executable="explore_node", name="hwr_explore",
             output="screen"),
        Node(package="hwr_gateway", executable="gateway_node", name="hwr_gateway",
             output="screen"),
    ])
