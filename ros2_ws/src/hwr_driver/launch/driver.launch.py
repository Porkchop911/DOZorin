import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("hwr_driver"),
                          "config", "driver.yaml")
    return LaunchDescription([
        Node(
            package="hwr_driver",
            executable="driver_node",
            name="hwr_driver",
            parameters=[config],
            output="screen",
        ),
    ])
