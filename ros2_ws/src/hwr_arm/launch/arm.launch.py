from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="hwr_arm", executable="arm_node", name="hwr_arm",
             output="screen"),
    ])
