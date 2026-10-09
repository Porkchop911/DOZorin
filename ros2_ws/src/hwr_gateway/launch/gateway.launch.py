from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="hwr_gateway", executable="gateway_node", name="hwr_gateway",
             output="screen"),
    ])
