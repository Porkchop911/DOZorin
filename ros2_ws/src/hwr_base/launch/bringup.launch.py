"""Minimal Phase-3 bringup: driver + base pipeline + robot_state_publisher."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    driver_launch = os.path.join(get_package_share_directory("hwr_driver"),
                                 "launch", "driver.launch.py")
    display_launch = os.path.join(get_package_share_directory("hwr_description"),
                                  "launch", "display.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(driver_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(display_launch)),
        Node(package="hwr_base", executable="base_node", name="hwr_base",
             output="screen"),
    ])
