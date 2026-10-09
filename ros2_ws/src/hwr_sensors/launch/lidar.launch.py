"""Lidar under our stack (plan 4.1, ADR-006): the installed sclidar_ros2
binary publishes scan_raw in lidar_frame (our URDF supplies the 180°
mount TF), our scan_filter_node crops it to the usable sector on scan.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port", default="/dev/lidar")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=port),
        Node(
            package="sclidar_ros2",
            executable="sclidar",
            name="hwr_lidar",
            parameters=[{
                "port": port,
                "baudrate": 230400,
                "version": 4,
                "frame_id": "lidar_frame",
            }],
            remappings=[("/scan", "scan_raw")],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="hwr_sensors",
            executable="scan_filter_node",
            name="hwr_scan_filter",
            output="screen",
        ),
    ])
