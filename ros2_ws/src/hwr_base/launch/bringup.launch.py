"""Minimal Phase-3 bringup: driver + base pipeline + robot_state_publisher."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    driver_launch = os.path.join(get_package_share_directory("hwr_driver"),
                                 "launch", "driver.launch.py")
    display_launch = os.path.join(get_package_share_directory("hwr_description"),
                                  "launch", "display.launch.py")
    calibration = os.path.join(get_package_share_directory("hwr_base"),
                               "config", "calibration.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("model_tf", default_value="true"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(driver_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(display_launch)),
        Node(package="hwr_base", executable="base_node", name="hwr_base",
             parameters=[calibration, {"publish_tf": LaunchConfiguration("model_tf")}],
             output="screen"),
    ])
