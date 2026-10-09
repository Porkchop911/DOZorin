"""Aurora 930 wrapper with stable HWR topic names and pointcloud disabled."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="deptrum-ros-driver-aurora930",
            executable="aurora930_node",
            name="hwr_depth_camera",
            parameters=[{
                "rgb_enable": True,
                # The closed driver publishes depth intrinsics on its IR
                # camera-info channel, so keep metadata enabled.
                "ir_enable": True,
                "depth_enable": True,
                "rgbd_enable": True,
                "point_cloud_enable": False,
                "rgb_fps": 12,
                "depth_correction": True,
                "align_mode": True,
                "minimum_filter_depth_value": 150,
                "maximum_filter_depth_value": 3000,
                "resolution_mode_index": 2,
            }],
            remappings=[
                ("depth/image_raw", "camera/depth/image_raw"),
                ("ir/camera_info", "camera/depth/camera_info"),
                ("rgb/camera_info", "camera/color/camera_info"),
                ("rgb/image_raw", "camera/color/image_raw"),
            ],
            output="screen",
            respawn=True,
            # Leave enough time for the stop-first supervisor's targeted USB
            # reset before the closed driver tries to reopen the device.
            respawn_delay=8.0,
        ),
        Node(package="hwr_sensors", executable="depth_to_ranges_node",
             name="hwr_depth_to_ranges", output="screen"),
    ])
