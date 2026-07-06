# Jetson-tuned RTAB-Map mapping, NODES ONLY (runs on top of the already-running base stack).
# External wheel/EKF odom (no visual odometry), lidar builds the grid, camera = loop closure.
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'qos_scan': qos, 'qos_image': qos,
        'queue_size': 30,
        'Reg/Strategy': '1',            # ICP (lidar) for link transforms
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',
        'Grid/Sensor': '0',            # occupancy grid from LIDAR only (clean); camera = loop closure
        'Grid/RangeMin': '0.2',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'Rtabmap/DetectionRate': '1.0',   # 1 keyframe/sec (Jetson budget)
        'Mem/ImagePreDecimation': '2',    # halve images for feature compute
        'Mem/ImagePostDecimation': '2',
        'Kp/MaxFeatures': '400',
    }
    remappings = [
        ('/tf', 'tf'), ('/tf_static', 'tf_static'),
        ('rgb/image', '/depth_cam/rgb0/image_raw'),
        ('rgb/camera_info', '/depth_cam/rgb0/camera_info'),
        ('depth/image', '/depth_cam/depth0/image_raw'),
        ('odom', '/odom'),
        ('scan', '/scan'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='2'),
        Node(package='rtabmap_sync', executable='rgbd_sync', output='screen',
             parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.02, 'qos': qos}],
             remappings=remappings),
        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=[parameters], remappings=remappings, arguments=['-d']),
    ])
