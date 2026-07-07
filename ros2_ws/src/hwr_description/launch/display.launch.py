import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory("hwr_description"),
                        "urdf", "hwr.urdf")
    with open(urdf) as f:
        robot_description = f.read()

    return LaunchDescription([
        # on the real robot hwr_driver publishes joint_states from servo
        # read-back; use fake:=true on a PC/VM without hardware
        DeclareLaunchArgument("fake", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("fake")),
        ),
    ])
