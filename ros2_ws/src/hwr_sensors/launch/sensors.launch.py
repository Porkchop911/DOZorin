"""Complete Phase-4 sensor graph: lidar, camera/depth bumper, and IMU filter."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hwr_sensors")
    include = lambda name: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", name)))
    return LaunchDescription([
        include("lidar.launch.py"),
        include("camera.launch.py"),
        Node(package="hwr_sensors", executable="imu_filter_node",
             name="hwr_imu_filter", output="screen"),
        Node(package="hwr_sensors", executable="sensor_supervisor_node",
             name="hwr_sensor_supervisor", output="screen"),
    ])
